#ifndef OMNI3_OMNI3_H
#define OMNI3_OMNI3_H

#include <Arduino.h>
#include <EEPROM.h>

#include "wheel.h"
#include "movements.h"
#include "motor_drivers/MDD3A.h"
#include "motor_drivers/MR001004.h"

/**
 * Omni3 is a 3-wheel drive robot and the number of wheels is of course 3
 */
#define WHEELS_NUM 3

/**
 * Looking the robot from the top and placing an imaginary clock dial where the positive forward direction of the robot
 * is at 12 o'clock, W_RIGHT is the wheel at 2 o'clock
 */
#define W_RIGHT 0

/**
 * Looking the robot from the top and placing an imaginary clock dial where the positive forward direction of the robot
 * is at 12 o'clock, W_BACK is the wheel at 6 o'clock
 */
#define W_BACK 1

/**
 * Looking the robot from the top and placing an imaginary clock dial where the positive forward direction of the robot
 * is at 12 o'clock, W_LEFT is the wheel at 10 o'clock
 */
#define W_LEFT 2

/**
 * Tangent of 30 degrees, or PI/6
 */
#define TAN30 0.57735027

/**
 * Cosine of 30 degrees, or PI/6
 */
#define COS30 0.86602540

/**
 * Cosine of 60 degrees, or PI/3
 */
#define COS60 0.5

/**
 * Cosine of 180 degrees, or PI
 */
#define COS180 -1.0

/**
 * Struct containing all information necessary for instancing a Omni3 object
 */
typedef struct omni3_params_s {
    /**
     * Wheels' radius in meters
     */
    double wheelsRadius;

    /**
     * Robot's radius (i.e. the distance between the center of the robot and a wheel) in meters
     */
    double robotRadius;

    /**
     * Wheels' maximum angular speed in radians per second
     */
    double maxWheelSpeed;

    /**
     * PID constants
     */
    double kP, kI, kD;
} omni3_params_t;

class Omni3 {
public:
    /**
     * Omni3 constructor, receiving as argument 3 Wheels objects and a parameters structure
     * @param rightWheel    pointer to the Wheel object, that handles the wheel at 2 o'clock
     * @param backWheel     pointer to the Wheel object, that handles the wheel at 6 o'clock
     * @param leftWheel     pointer to the Wheel object, that handles the wheel at 10 o'clock
     * @param parameters    pointer to the omni3_params_t with the desired information
     */
    Omni3(Wheel* rightWheel, Wheel* backWheel, Wheel* leftWheel, Movements* movements, omni3_params_t parameters) {
        /* Set array of Wheel pointers */
        wheels[W_RIGHT] = rightWheel;
        wheels[W_BACK] = backWheel;
        wheels[W_LEFT] = leftWheel;

        /* Set movements pointer */
        this->movements = movements;

        /* Set wheels and robot radius*/
        this->setWheelsRadius(parameters.wheelsRadius);
        this->setRobotRadius(parameters.robotRadius);

        /* For each wheel, set max speed and PID constants */
        for (auto & wheel : wheels) {
            wheel->setMaxSpeed(parameters.maxWheelSpeed);
            wheel->setPID(parameters.kP, parameters.kI, parameters.kD);
        }
    }

    /**
     * Omni3 constructor, receiving as argument 3 Wheels objects and a memory address for reading parameters
     * @param rightWheel    pointer to the Wheel object, that handles the wheel at 2 o'clock
     * @param backWheel     pointer to the Wheel object, that handles the wheel at 6 o'clock
     * @param leftWheel     pointer to the Wheel object, that handles the wheel at 10 o'clock
     * @param memAddr       starting memory address where data is stored
     */
    Omni3(Wheel* rightWheel, Wheel* backWheel, Wheel* leftWheel, Movements* movements, int memAddr) :
            Omni3(rightWheel, backWheel, leftWheel, movements, Omni3::readStoredData(memAddr)) {}

    /**
     * This method asynchronously handles the movement of the robot: it must be called inside the main Arduino loop
     */
    void handle();

    /**
     * This static method reads from memory omni3 parameters
     * @param memAddr   starting memory address where data is stored
     * @return parameters read
     */
    static omni3_params_t readStoredData(int memAddr);

    /**
     * This method sets both current and target position to 0, if the robot is still
     * @return true if positions were reset, false otherwise
     */
    bool home();

    /**
     * This method instantly stops the robot; Arduino reset is needed in order to make the robot work again
     */
    void emergencyStop();

private:
    /**
     * Wheels array elements are ordered as follows: looking the robot from the top, place an imaginary clock dial where
     * the positive forward direction of the robot is at 12 o'clock; proceeding clockwise:
     * - wheels[W_RIGHT] Wheel will be at 2 o'clock
     * - wheels[W_BACK] Wheel will be at 6 o'clock
     * - wheels[W_LEFT] Wheel will be at 10 o'clock
     */
    Wheel* wheels[WHEELS_NUM]{};

    /**
     *
     */
    Movements* movements;

    /**
     * Vector storing the current position of the robot: currentPosition[X]: meters, currentPosition[Y]: meters,
     * currentPosition[TH]: radians; positions are reset when home() method is called
     */
    double currentPosition[DOF] = {0.0, 0.0, 0.0};

    /**
     * Vector storing the target position of the robot: targetPosition[X]: meters, targetPosition[Y]: meters,
     * targetPosition[TH]: radians; positions are reset when home() method is called
     */
    double targetPosition[DOF] = {0.0, 0.0, 0.0};

    /**
     * Vector storing the current movement of the robot: displacement[FORWARD]: meters, displacement[STRAFE]: meters,
     * displacement[ANGULAR]: radians; home() can be called only if displacement is 0; the current speed of the robot
     * can be easily computed by dividing the displacement by the time elapsed
     */
    double displacement[DOF] = {0.0, 0.0, 0.0};

    /**
     * Timestamp in milliseconds of last execution
     */
    unsigned long lastTime = 0;

    /**
     * Wheels' radius in meters
     */
    double R = 1.0;

    /**
     * Robot's radius (i.e. the distance between the center of the robot and a wheel) in meters
     */
    double L = 1.0;

    /**
     * Cosine of 30 degrees divided by wheels' radius
     */
    double C30_R = 1.0;

    /**
     * Cosine of 60 degrees divided by wheels' radius
     */
    double C60_R = 1.0;

    /**
     * Cosine of 180 degrees divided by wheels' radius
     */
    double C180_R = 1.0;

    /**
     * Tangent of 30 degrees times wheels' radius
     */
    double T30R = 1.0;

    /**
     * Wheels' radius divided by 3
     */
    double R_3 = 1.0;

    /**
     * Robot's radius divided by wheels' radius
     */
    double L_R = 1.0;

    /**
     * Wheels' radius divided by 3 times robot's radius
     */
    double R_3L = 1.0;

    /**
     * This method sets the wheel radius and computes all related constants
     * @param wheelsRadius  radius of the wheel in meters
     */
    void setWheelsRadius(double wheelsRadius);

    /**
     * This method sets the robot radius and computes all related constants
     * @param robotRadius   radius of the robot in meters
     */
    void setRobotRadius(double robotRadius);

    /**
     * This method computes the robot displacement, given each wheel's displacement
     * @param angularDisplacement   array of wheels' angular displacements in radians
     */
    void directKinematics(const double* angularDisplacement);

    /**
     * This method computes and sets wheels' angular speeds, given the desired robot speed vector
     * @param speed     array of speeds: speed[FORWARD]: m/s, speed[STRAFE]: m/s, speed[THETA]: rad/s
     * @returns true if the movement is feasible, false otherwise
     */
    bool inverseKinematics(const double* speed) const;

    /**
     * This method computes and sets robot's current position from the current position and the displacement
     */
    void odometry();

    /**
     * This method returns the minimum angular distance between two angular positions
     * @param phi1    first angle in radians; it must be in range [0, PI)
     * @param phi2    second angle in radians; it must be in range [0, PI)
     * @return angular distance in radians; it will be in range [0, PI]
     */
    static double angularDistance(double phi1, double phi2);

};


#endif
