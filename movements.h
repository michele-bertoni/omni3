#ifndef OMNI3_MOVEMENTS_H
#define OMNI3_MOVEMENTS_H

#include "Arduino.h"

/**
 * Omni3 robots have 2 possible vector frames, each of them having 3 degrees of freedom (DOF)
 * - Displacement vector's frame of reference is jointed with the robot: (FORWARD, STRAFE, THETA)
 * - Position vector's frame of reference is disjoint from the robot: (POS_X, POS_Y, POS_PHI)
 */
#define DOF 3

/**
 * FORWARD is oriented in the opposite direction than the BACK wheel
 */
#define FORWARD 0

/**
 * STRAFE is normal to FORWARD vector and oriented towards the LEFT of the robot
 */
#define STRAFE 1

/**
 * THETA is oriented anti-clockwise, looking the robot from the top
 */
#define THETA 2

/**
 * When POS_X is homed, it overlaps FORWARD vector
 */
#define POS_X 0

/**
 * When POS_Y is homed, it overlaps STRAFE vector
 */
#define POS_Y 1

/**
 * When POS_PHI is homed, it overlaps ANGULAR vector
 */
#define POS_PHI 2

/**
 * Seconds in one millisecond
 */
#define MILLIS 0.001

/**
 * Milliseconds in one second
 */
#define TO_MILLIS 1000

/**
 * This class handles movements of the robot
 */
class Movements {
private:
    /**
     * This abstract class describes a generic movement
     */
    class Movement {
    public:
        /**
         * Pure virtual destructor
         */
        virtual ~Movement() = 0;

        /**
         * This pure virtual method must be overridden by a method that, given the current position and the time,
         * sets targetSpeed values to the desired ones
         * @param time          current time in milliseconds
         * @param targetSpeed   array in which target speed vector ([m/s, m/s, rad/s]) is stored
         */
        virtual void getSpeed(unsigned long time, double *targetSpeed) = 0;
    };

    /**
     * This abstract class describes all movements that have a defined end
     */
    class FiniteMovement : public Movement {
    public:
        /**
         * Pure virtual destructor
         */
        ~FiniteMovement() override = 0;

        /**
         * This pure virtual method must be overridden by a method that, given the current position, the space of
         * brake and the time, sets _isFinished values to true if the current movement ended for the given axes,
         * false otherwise; if _isFinished is all composed by true values, movements schedule is shifted
         * @param position      current position of the robot ([m, m, rad])
         * @param brakingSpace  space needed by the robot to stop ([m, m, rad])
         * @param time          current time in milliseconds
         * @return true if all movement components finished, false otherwise
         */
        virtual bool isFinished(double *position, double *brakingSpace, unsigned long time) = 0;

    protected:
        /**
         * This method computes and returns the timestamp when the movement is supposed to end, given the timestamp
         * when the movement began and the duration of the movement
         * @param time          current time in milliseconds
         * @param duration      duration of the movement in seconds
         * @return target timestamp in milliseconds when the movement is supposed to end
         */
        static unsigned long initializeTargetTime(unsigned long time, double duration) {
            /* Compute target time */
            unsigned long targetTime = time + lround(duration * TO_MILLIS);

            /* Since a target time of 0 is seen as uninitialized, if result is by chance 0, it is set to 1 (1 ms of
               difference is very small, also it is unlikely that the result is 0); finally return target time */
            return targetTime==0 ? 1 : targetTime;
        }

        /**
         * This method computes the conversion from (POS_X, POS_Y) coordinates to (FORWARD, STRAFE) coordinates
         * @param x         x component of input vector
         * @param y         y component of input vector
         * @param phi       angle in radians (positive anti-clockwise) from x to new_x
         * @param new_x     x component of output vector
         * @param new_y     y component of output vector
         */
        static void xyToSF(double x, double y, double phi, double *new_x, double *new_y) {
            /* x' = x*cos(phi) + y*sin(phi) */
            *new_x = x*cos(phi) + y*sin(phi);

            /* y' = -x*sin(phi) + y*cos(phi) */
            *new_y = -x*sin(phi) + y*cos(phi);
        }

        /**
         * Vector containing information about whether the single component of speed vector ended or not
         */
        bool _isFinished[DOF] = {false, false, false};

        /**
         * Tolerance in meters
         */
        constexpr static double linearTolerance = 0.01;

        /**
         * Tolerance in radians
         */
        constexpr static double angularTolerance = 0.0174533;
    };

    /**
     * This abstract class describes all movements that can possibly last forever
     */
    class IndefiniteMovement : public Movement { ;
    public:
        /**
         * Pure virtual destructor
         */
        ~IndefiniteMovement() override = 0;

        /**
         * Pure virtual method for deleting the object: if the object can be deleted, override this method with
         * "delete this;", otherwise override it with an empty method
         */
        virtual void free() = 0;
    };

    /**
     * This class describes the indefinite movement of staying still in the same position; it is the default
     * movement, so if no other movements are set, the robot will stay still; for this reason, this class is
     * a singleton, so that it won't be instanced and deleted everytime
     */
    class Still : public IndefiniteMovement {
    public:
        /**
         * This method, independently from the arguments, will set targetSpeed to [0.0, 0.0, 0.0]
         * @param position      current position of the robot ([m, m, rad])
         * @param time          current time in milliseconds
         * @param targetSpeed   array in which target speed vector ([m/s, m/s, rad/s]) is stored
         */
        void getSpeed(unsigned long time, double *targetSpeed) override {
            targetSpeed[FORWARD] = 0.0;
            targetSpeed[STRAFE] = 0.0;
            targetSpeed[THETA] = 0.0;
        }

        /**
         * This method does nothing, since a Singleton can't be deleted
         */
        void free() override {}

        /**
         * This method returns a pointer, that is the instance of the Still singleton
         * @return the instance of Still singleton
         */
        static Still* getInstance() {
            static auto* instance = new Still;
            return instance;
        }

        /**
         * Delete default constructor that makes a copy of a Still object
         */
        Still(Still const&) = delete;

        /**
         * Delete assign operator
         */
        void operator = (Still const&) = delete;

    private:
        /**
         * Constructor is made private for avoiding creation of other instances outside the one of the Singleton
         */
        Still() = default;
    };

    /**
     * This class describes the finite movement that brings the robot to the requested position at the given time
     */
    class SpaceTimeLinear : public FiniteMovement {
    public:
        /**
         * Constructor of the movement described by space and time
         * @param x         target x coordinate in meters
         * @param y         target y coordinate in meters
         * @param phi       target phi coordinate in radians
         * @param duration  duration of the movement in seconds
         */
        SpaceTimeLinear(double x, double y, double phi, double duration) {
            this->target[POS_X] = x;
            this->target[POS_Y] = y;
            this->target[POS_PHI] = phi;
            this->_duration = duration;
            this->targetTime = 0;
        }

        /**
         * This override method, given the current position and the time, sets targetSpeed vector values
         * so that the achieved trajectory is linear in all of its components
         * @param time          current time in milliseconds
         * @param targetSpeed   array in which target speed vector ([m/s, m/s, rad/s]) is stored
         */
        void getSpeed(unsigned long time, double *targetSpeed) override {
            /* If targetTime is 0 (first call of this method on this object), initialize targetTime */
            if(this->targetTime == 0) {
                this->targetTime = FiniteMovement::initializeTargetTime(time, this->_duration);
            }

            /* Compute delta time in seconds */
            double dt = (targetTime - time) * MILLIS;

            /* Compute speed vectors as displacement over delta time in the FORWARD, STRAFE, THETA frame of reference;
               if that component movement is finished, speed is set to 0; this avoids oscillations */
            targetSpeed[FORWARD] = _isFinished[FORWARD] ? 0.0 : displacements[FORWARD] / dt;
            targetSpeed[STRAFE] = _isFinished[STRAFE] ? 0.0 : displacements[STRAFE] / dt;
            targetSpeed[THETA] = _isFinished[THETA] ? 0.0 : displacements[THETA] / dt;
        }

        /**
         * This override method, given the current position, the space of brake and the time, sets _isFinished
         * values to true if the current movement ended for the given axes, false otherwise;
         * if _isFinished is all composed by true values, movements schedule is shifted
         * @param position      current position of the robot ([m, m, rad])
         * @param brakingSpace  space needed by the robot to stop ([m, m, rad])
         * @param time          current time in milliseconds
         * @return true if all movement components finished, false otherwise
         */
        bool isFinished(double *position, double *brakingSpace, unsigned long time) override {
            /* Compute displacements in [FORWARD, STRAFE, THETA] frame of reference */
            FiniteMovement::xyToSF(target[POS_X]-position[POS_X], target[POS_Y]-position[POS_Y],
                                   position[POS_PHI], &displacements[FORWARD], &displacements[STRAFE]);
            displacements[THETA] = target[POS_PHI] - position[POS_PHI];

            /* If displacement is inside range of tolerance, i.e. the maximum between braking space (that depends on the
               square of current speed) and a constant tolerance, the movement on the corresponding axes is finished */
            this->_isFinished[FORWARD] =
                    (displacements[FORWARD] <= max(brakingSpace[FORWARD], linearTolerance)) &&
                    (displacements[FORWARD] >= -max(brakingSpace[FORWARD], linearTolerance));
            this->_isFinished[STRAFE] =
                    (displacements[STRAFE] <= max(brakingSpace[STRAFE], linearTolerance)) &&
                    (displacements[STRAFE] >= -max(brakingSpace[STRAFE], linearTolerance));
            this->_isFinished[THETA] =
                    (displacements[THETA] <= max(brakingSpace[THETA], angularTolerance)) &&
                    (displacements[THETA] >= -max(brakingSpace[THETA], angularTolerance));

            /* return true if the movement is completed for all the components */
            return _isFinished[FORWARD] && _isFinished[STRAFE] && _isFinished[THETA];
        }

    private:
        /**
         * Vector with target positions
         */
        double target[DOF] {};

        /**
         * Vector with displacements
         */
        double displacements[DOF] = {0.0, 0.0, 0.0};

        /**
         * Duration of the movement in seconds
         */
        double _duration;

        /**
         * Target time the movement should finish
         */
        unsigned long targetTime;
    };

    /**
     * This class describes the finite movement that brings the robot to the requested position with the given speeds
     */
    class SpaceSpeedLinear : public FiniteMovement {
    public:
        /**
         * Constructor of the movement described by space and time
         * @param x             target x coordinate in meters
         * @param y             target y coordinate in meters
         * @param phi           target phi coordinate in radians
         * @param speedMag      magnitude of planar speed vector in meters per second
         * @param angularMag    magnitude of angular speed vector in radians per second
         */
        SpaceSpeedLinear(double x, double y, double phi, double speedMag, double angularMag) {
            this->target[POS_X] = x;
            this->target[POS_Y] = y;
            this->target[POS_PHI] = phi;
            this->sqrtSpeedMagnitude = sqrt(speedMag);
            this->angularSpeedMagnitude = angularMag;
        }

        /**
         * This override method, given the current position and the time, sets targetSpeed vector values
         * so that the achieved trajectory is linear in all of its components
         * @param time          current time in milliseconds
         * @param targetSpeed   array in which target speed vector ([m/s, m/s, rad/s]) is stored
         */
        void getSpeed(unsigned long time, double *targetSpeed) override {
            /* Compute normalization factor, to be multiplied to each component, in order to have a resulting speed
               vector with the given magnitude, and the same direction as the displacement vector */
            double normFactor = sqrtSpeedMagnitude /
                    sqrt(displacements[FORWARD]*displacements[FORWARD] + displacements[STRAFE]*displacements[STRAFE]);
            targetSpeed[FORWARD] = _isFinished[FORWARD] ? 0.0 : displacements[FORWARD] * normFactor;
            targetSpeed[STRAFE] = _isFinished[STRAFE] ? 0.0 : displacements[STRAFE] * normFactor;

            /* Compute the angular speed, by multiplying the desired magnitude to the sign of the displacement */
            targetSpeed[THETA] = _isFinished[THETA] ? 0.0 : (displacements[THETA]>=0 ? 1 : -1)*angularSpeedMagnitude;
        }

        /**
         * This override method, given the current position, the space of brake and the time, sets _isFinished
         * values to true if the current movement ended for the given axes, false otherwise;
         * if _isFinished is all composed by true values, movements schedule is shifted
         * @param position      current position of the robot ([m, m, rad])
         * @param brakingSpace  space needed by the robot to stop ([m, m, rad])
         * @param time          current time in milliseconds
         * @return true if all movement components finished, false otherwise
         */
        bool isFinished(double *position, double *brakingSpace, unsigned long time) override {
            /* Compute displacements in [FORWARD, STRAFE, THETA] frame of reference */
            FiniteMovement::xyToSF(target[POS_X]-position[POS_X], target[POS_Y]-position[POS_Y],
                                   position[POS_PHI], &displacements[FORWARD], &displacements[STRAFE]);
            displacements[THETA] = target[POS_PHI] - position[POS_PHI];

            /* If displacement is inside range of tolerance, i.e. the maximum between braking space (that depends on the
               square of current speed) and a constant tolerance, the movement on the corresponding axes is finished */
            this->_isFinished[FORWARD] =
                    (displacements[FORWARD] <= max(brakingSpace[FORWARD], linearTolerance)) &&
                    (displacements[FORWARD] >= -max(brakingSpace[FORWARD], linearTolerance));
            this->_isFinished[STRAFE] =
                    (displacements[STRAFE] <= max(brakingSpace[STRAFE], linearTolerance)) &&
                    (displacements[STRAFE] >= -max(brakingSpace[STRAFE], linearTolerance));
            this->_isFinished[THETA] =
                    (displacements[THETA] <= max(brakingSpace[THETA], angularTolerance)) &&
                    (displacements[THETA] >= -max(brakingSpace[THETA], angularTolerance));

            /* return true if the movement is completed for all the components */
            return _isFinished[FORWARD] && _isFinished[STRAFE] && _isFinished[THETA];
        }

    private:
        /**
         * Vector with target positions
         */
        double target[DOF] {};

        /**
         * Vector with displacements
         */
        double displacements[DOF] = {0.0, 0.0, 0.0};

        /**
         * Square root of the magnitude of the speed vector composed by forward and strafe vectors
         */
        double sqrtSpeedMagnitude;

        /**
         * Magnitude of the angular speed in rad/s
         */
        double angularSpeedMagnitude;

    };

    class TimeSpeedLinear : public FiniteMovement {
    public:
        void getSpeed(unsigned long time, double *targetSpeed) override {

        }

    };

    class SpeedIndefinite : public IndefiniteMovement {
    public:
        SpeedIndefinite(double forward, double strafe, double angular) {
            speed[FORWARD] = forward;
            speed[STRAFE] = strafe;
            speed[THETA] = angular;
        }

        void getSpeed(unsigned long time, double *targetSpeed) override {
            speed[FORWARD] = this->speed[FORWARD];
            speed[STRAFE] = this->speed[STRAFE];
            speed[THETA] = this->speed[THETA];
        }

        void free() override {
            delete this;
        }

    private:
        /**
         * Target forward, strafe and angular speeds
         */
        double speed[DOF]{};
    };

    #define MAX_MOVEMENTS 10

    FiniteMovement *movementsSchedule[MAX_MOVEMENTS] = { nullptr };

    IndefiniteMovement *defaultMovement = Still::getInstance();

    uint8_t freeIndex = 0;

public:
    void addStop() {
        this->defaultMovement->free();
        this->defaultMovement = Still::getInstance();
    }

    void addConstantMovement(double forward, double strafe, double angular) {
        this->defaultMovement->free();
        this->defaultMovement = new SpeedIndefinite(forward, strafe, angular);
    }

    bool addTargetPosTime(double x, double y, double phi, double duration) {
        addStop();
        this->movementsSchedule[freeIndex] = new SpaceTimeLinear(x, y, phi, duration);
    }
};

#endif //OMNI3_MOVEMENTS_H
