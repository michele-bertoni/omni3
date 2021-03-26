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
         * false otherwise; if _isFinished is all composed by true values, movements schedule is shifted;
         * for finite movement, it's important to call isFinished method before getSpeed method
         * @param position      current position of the robot ([m, m, rad])
         * @param brakingSpace  space needed by the robot to stop ([m, m, rad])
         * @param time          current time in milliseconds
         * @return true if all movement components finished, false otherwise
         */
        virtual bool isFinished(const double *position, const double *brakingSpace, unsigned long time) = 0;

    protected:
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
            this->startTime = 0;
        }

        /**
         * This overriding method, given the current position, the space of brake and the time, sets _isFinished
         * values to true if the current movement ended for the given axes, false otherwise;
         * if _isFinished is all composed by true values, movements schedule is shifted
         * @param position      current position of the robot ([m, m, rad])
         * @param brakingSpace  space needed by the robot to stop ([m, m, rad])
         * @param time          current time in milliseconds
         * @return true if all movement components finished, false otherwise
         */
        bool isFinished(const double *position, const double *brakingSpace, unsigned long time) override {
            /* If targetTime is 0 (first call of this method on this object), initialize targetTime */
            if(this->startTime == 0) {
                /* Since a target time of 0 is seen as uninitialized, if result is by chance 0, it is set to 1 (1 ms of
                   difference is very small, also it is unlikely that the result is 0) */
                this->startTime = time!=0 ? time : 1;
            }

            /* If current duration of the movement is at least the theoretical duration, end current movement */
            if((time - startTime) >= lround(_duration * TO_MILLIS)) {
                return true;
            }

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

        /**
         * This overriding method, given the displacements and the time, sets targetSpeed vector values
         * so that the achieved trajectory is linear in all of its components
         * @param time          current time in milliseconds
         * @param targetSpeed   array in which target speed vector ([m/s, m/s, rad/s]) is stored
         */
        void getSpeed(unsigned long time, double *targetSpeed) override {
            /* Compute delta time in seconds */
            double dt = _duration - ((time - startTime) * MILLIS);

            /* Compute speed vectors as displacement over delta time in the FORWARD, STRAFE, THETA frame of reference;
               if that component movement is finished, speed is set to 0; this avoids oscillations */
            targetSpeed[FORWARD] = _isFinished[FORWARD] ? 0.0 : displacements[FORWARD] / dt;
            targetSpeed[STRAFE] = _isFinished[STRAFE] ? 0.0 : displacements[STRAFE] / dt;
            targetSpeed[THETA] = _isFinished[THETA] ? 0.0 : displacements[THETA] / dt;
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
         * Time when then movement started
         */
        unsigned long startTime;
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
         * This overriding method, given the current position, the space of brake and the time, sets _isFinished
         * values to true if the current movement ended for the given axes, false otherwise;
         * if _isFinished is all composed by true values, movements schedule is shifted
         * @param position      current position of the robot ([m, m, rad])
         * @param brakingSpace  space needed by the robot to stop ([m, m, rad])
         * @param time          current time in milliseconds
         * @return true if all movement components finished, false otherwise
         */
        bool isFinished(const double *position, const double *brakingSpace, unsigned long time) override {
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

        /**
         * This overriding method, given the displacements and speeds' magnitude, sets targetSpeed vector values
         * so that the achieved trajectory is linear in all of its components (but together they might be non-linear)
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

    /**
     * This class describes the finite movement that makes the robot move with the requested speeds for the given time
     */
    class SpeedTimeLinear : public FiniteMovement {
    public:
        /**
         * Constructor of the movement described by space and time
         * @param forward   forward speed in m/s
         * @param strafe    strafe speed in m/s
         * @param angular   angular speed in rad/s
         * @param duration  duration of the movement in seconds
         */
        SpeedTimeLinear(double forward, double strafe, double angular, double duration) {
            this->speed[FORWARD] = forward;
            this->speed[STRAFE] = strafe;
            this->speed[THETA] = angular;
            this->_duration = duration;
            this->startTime = 0;
        }

        /**
         * This overriding method, given the current position, the space of brake and the time, sets _isFinished
         * values to true if the current movement ended for the given axes, false otherwise;
         * if _isFinished is all composed by true values, movements schedule is shifted
         * @param position      current position of the robot ([m, m, rad])
         * @param brakingSpace  space needed by the robot to stop ([m, m, rad])
         * @param time          current time in milliseconds
         * @return true if all movement components finished, false otherwise
         */
        bool isFinished(const double *position, const double *brakingSpace, unsigned long time) override {
            /* If targetTime is 0 (first call of this method on this object), initialize targetTime */
            if(this->startTime == 0) {
                /* Since a target time of 0 is seen as uninitialized, if result is by chance 0, it is set to 1 (1 ms of
                   difference is very small, also it is unlikely that the result is 0) */
                this->startTime = time!=0 ? time : 1;
            }

            /* If current duration of the movement is at least the theoretical duration, end current movement */
            if((time - startTime) >= lround(_duration * TO_MILLIS)) {
                return true;
            }
            return false;
        }

        /**
         * This overriding method, given the target speeds, sets targetSpeed vector values to the requested ones
         * @param time          current time in milliseconds
         * @param targetSpeed   array in which target speed vector ([m/s, m/s, rad/s]) is stored
         */
        void getSpeed(unsigned long time, double *targetSpeed) override {
            /* Speed vector components are simply equal to the requested speeds */
            targetSpeed[FORWARD] = this->speed[FORWARD];
            targetSpeed[STRAFE] = this->speed[STRAFE];
            targetSpeed[THETA] = this->speed[THETA];
        }

    private:
        /**
         * Vector with target positions
         */
        double speed[DOF] {};

        /**
         * Duration of the movement in seconds
         */
        double _duration;

        /**
         * Target time the movement should finish
         */
        unsigned long startTime;

    };

    /**
     * This class describes the indefinite movement that makes the robot move with the requested speeds
     */
    class SpeedIndefinite : public IndefiniteMovement {
    public:
        /**
         * Constructor sets speeds values
         * @param forward   forward component of speed vector
         * @param strafe    strafe component of speed vector
         * @param angular   angular component of speed vector
         */
        SpeedIndefinite(double forward, double strafe, double angular) {
            speed[FORWARD] = forward;
            speed[STRAFE] = strafe;
            speed[THETA] = angular;
        }

        /**
         * This overriding method, given the target speeds, sets targetSpeed vector values to the requested ones
         * @param time          current time in milliseconds
         * @param targetSpeed   array in which target speed vector ([m/s, m/s, rad/s]) is stored
         */
        void getSpeed(unsigned long time, double *targetSpeed) override {
            targetSpeed[FORWARD] = this->speed[FORWARD];
            targetSpeed[STRAFE] = this->speed[STRAFE];
            targetSpeed[THETA] = this->speed[THETA];
        }

        /**
         * Overriding method for deleting the object
         */
        void free() override {
            delete this;
        }

    private:
        /**
         * Target forward, strafe and angular speeds
         */
        double speed[DOF]{};
    };

    /**
     * Define max number of movements in movementSchedule array
     */
    #define MAX_MOVEMENTS 10

    /**
     * Array of scheduled movements
     */
    FiniteMovement *movementsSchedule[MAX_MOVEMENTS] = { nullptr };

    /**
     * Movement performed if *movementSchedule[0] is equal to nullptr; it's set to Still every time a new FiniteMovement
     * is scheduled; so there is no need to call addStop(), unless last scheduled movement was an IndefiniteMovement
     */
    IndefiniteMovement *defaultMovement = Still::getInstance();

    /**
     * Index of the first free element of the array
     */
    uint8_t freeIndex = 0;

    /**
     * Coefficients that, multiplied by the square of the corresponding speed component, returns the braking space
     */
    double frictionCoefficient[DOF]{};

    /**
     * This method sets the given IndefiniteMovement as current
     * @param indefiniteMovement    movement to be set
     */
    void setIndefiniteMovement(IndefiniteMovement *indefiniteMovement) {
        /* Unschedule current IndefiniteMovement */
        this->defaultMovement->free();

        /* Set current indefinite movement */
        this->defaultMovement = indefiniteMovement;
    }

    /**
     * This method appends the given FiniteMovement to the schedule
     * @param finiteMovement        movement to be appended
     * @return true if movement was appended, false otherwise
     */
    bool appendFiniteMovement(FiniteMovement *finiteMovement) {
        /* If schedule is full, delete finiteMovement object and return false */
        if(freeIndex >= MAX_MOVEMENTS) {
            delete finiteMovement;
            return false;
        }

        /* Unschedule IndefiniteMovement that may be instanced */
        addStop();

        /* Otherwise, place finiteMovement in the schedule, increment freeIndex and return true */
        this->movementsSchedule[freeIndex] = finiteMovement;
        freeIndex++;
        return true;
    }

public:
    /**
     * Public constructor for instancing an object of class Movements
     * @param forwardFrictionK  coefficient of friction on forward component of speed vector
     * @param strafeFrictionK   coefficient of friction on strafe component of speed vector
     * @param angularFrictionK  coefficient of friction on angular component of speed vector
     */
    Movements(double forwardFrictionK, double strafeFrictionK, double angularFrictionK) {
        this->frictionCoefficient[FORWARD] = forwardFrictionK;
        this->frictionCoefficient[STRAFE] = strafeFrictionK;
        this->frictionCoefficient[THETA] = angularFrictionK;
    }

    /**
     * Public constructor for instancing an object of class Movements without braking space compensation
     */
    Movements() : Movements(0.0, 0.0, 0.0) {}

    /**
     * Schedules a Still movement; this method is called every time a new FiniteMovement is scheduled, so there is no
     * need to manually call it in order to schedule a stop, unless if last scheduled movement was an indefinite one
     */
    void addStop() {
        this->setIndefiniteMovement(Still::getInstance());
    }

    /**
     * Schedules an indefinite movement with given forward, strafe and angular speeds
     * @param forward   requested forward speed's magnitude
     * @param strafe    requested strafe speed's magnitude
     * @param angular   requested angular speed's magnitude
     */
    void addConstantMovement(double forward, double strafe, double angular) {
        this->setIndefiniteMovement(new SpeedIndefinite(forward, strafe, angular));
    }

    /**
     * Schedule a finite movement that ends when position is reached or time limit is reached, whatever comes first
     * @param x         target x position
     * @param y         target y position
     * @param phi       target phi position
     * @param duration  duration of the movement
     * @return boolean indicating whether the movement was scheduled or not (i.e. because schedule is full)
     */
    bool addTargetPosTime(double x, double y, double phi, double duration) {
        return this->appendFiniteMovement(new SpaceTimeLinear(x, y, phi, duration));
    }

    /**
     * Schedule a finite movement that ends when position is reached or time limit is reached, whatever comes first
     * @param x             target x position
     * @param y             target y position
     * @param phi           target phi position
     * @param speedMag      positive magnitude of planar speed vector
     * @param angularMag    positive magnitude of angular speed vector
     * @return boolean indicating whether the movement was scheduled or not (i.e. because schedule is full)
     */
    bool addTargetPosSpeed(double x, double y, double phi, double speedMag, double angularMag) {
        return this->appendFiniteMovement(new SpaceSpeedLinear(x, y, phi, speedMag, angularMag));
    }

    /**
     * Schedule a finite movement that ends when time limit is reached
     * @param forward   requested forward speed's magnitude
     * @param strafe    requested strafe speed's magnitude
     * @param angular   requested angular speed's magnitude
     * @param duration  duration of the movement
     * @return boolean indicating whether the movement was scheduled or not (i.e. because schedule is full)
     */
    bool addTargetSpeedTime(double forward, double strafe, double angular, double duration) {
        return this->appendFiniteMovement(new SpeedTimeLinear(forward, strafe, angular, duration));
    }

    /**
     * This method handles the movements: it must be called frequently, passing updated requested arguments
     * @param currentPosition   current position of the robot ([m, m, rad])
     * @param currentSpeed      current speed of the robot ([m/s, m/s, rad/s])
     * @param time              current time in milliseconds
     * @param targetSpeed       array in which this method sets computed speed vector ([FORWARD, STRAFE, THETA])
     */
    void handle(const double *currentPosition, const double *currentSpeed, unsigned long time, double *targetSpeed) {
        /* If there are no scheduled movements, perform the default indefinite movement */
        if (freeIndex <= 0) {
            this->defaultMovement->getSpeed(time, targetSpeed);
            return;
        }

        /* Compute brakingSpace given current speed and friction coefficients vectors */
        double brakingSpace[DOF];
        brakingSpace[FORWARD] = currentSpeed[FORWARD] * currentSpeed[FORWARD] * frictionCoefficient[FORWARD];
        brakingSpace[STRAFE] = currentSpeed[STRAFE] * currentSpeed[STRAFE] * frictionCoefficient[STRAFE];
        brakingSpace[THETA] = currentSpeed[THETA] * currentSpeed[THETA] * frictionCoefficient[THETA];

        /* While current movement is finished, shift schedule of one position */
        while(this->movementsSchedule[0]->isFinished(currentPosition, brakingSpace, time)) {
            delete movementsSchedule[0];
            for(int i=0; i<freeIndex-1; i++) {
                movementsSchedule[i] = movementsSchedule[i+1];
            }
            freeIndex--;
            movementsSchedule[freeIndex] = nullptr;
        }

        /* If there are no scheduled movements, perform the default indefinite movement,
           otherwise perform current finite movement */
        freeIndex <= 0 ? this->defaultMovement->getSpeed(time, targetSpeed) :
                this->movementsSchedule[0]->getSpeed(time, targetSpeed);
    }

    /**
     * Undefine max number of movements so that this define is kept private
     */
    #undef MAX_MOVEMENTS
};

#endif //OMNI3_MOVEMENTS_H
