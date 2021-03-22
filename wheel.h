#ifndef OMNI3_WHEEL_H
#define OMNI3_WHEEL_H

#include "Arduino.h"
#include "Encoder.h"
#include "motor_driver.h"

#define D_KP 1.4f
#define D_KI 0.5f
#define D_KD 0.8f

#define MICROS 0.000001f

/**
 * Class for defining and controlling wheels
 */
class Wheel {
public:
    /**
     * Wheel constructor
     * @param driver    Driver for handling the wheel; it must be an instance of a child class of the class MotorDriver
     * @param encoder   Encoder for reading wheel position
     */
    Wheel(const MotorDriver& driver, const Encoder& encoder) :
            driver(driver), encoder(encoder), maxSpeed(0) {
        /* Initialize PID constants and initialize speed to 0 */
        this->setDefaultPID();
        this->setNormalizedSpeed(0);
    }

    /**
     * This method sets PID constants to the given parameters
     * @param kP        Proportional constant
     * @param kI        Integrative constant
     * @param kD        Derivative constant
     */
    void setPID(float kP, float kI, float kD) {
        this->kP = kP;
        this->kI = kI;
        this->kD = kD;
    }

    /**
     * This method sets PID constants to the default ones stored in defines D_KP, D_KI and D_KD
     */
    void setDefaultPID() {
        setPID(D_KP, D_KI, D_KD);
    }

    /**
     * This method sets the target speed of the motor and returns how many steps it moved from last time it was called
     * @param speed     requested speed in radians per second
     * @return number of radians the wheel rotated since last call of this function
     * @throws invalid_argument if normSpeed's absolute value is greater than maximum angular speed of the wheel
     */
    float setSpeed(float speed) {
        if(maxSpeed == 0) {
            return this->setNormalizedSpeed(0);
        }
        return this->setNormalizedSpeed(speed / this->maxSpeed);
    }

    /**
     * This method sets the target speed of the motor and returns how many steps it moved from last time it was called
     * @param normSpeed requested speed in range [-1, 1]
     * @return number of radians the wheel rotated since last call of this function
     * @throws invalid_argument if normSpeed is out of range or maxSpeed is zero
     */
    float setNormalizedSpeed(float normSpeed) {
        /* If requested speed is not zero, but maxSpeed is zero, throw an exception */
        if (normSpeed != 0.0 && this->maxSpeed == 0.0) {
            throw std::invalid_argument("Wheel's max speed is 0, set a proper maximum angular speed");
        }
        /* If requested speed is greater than maxSpeed, throw an exception */
        if (normSpeed > 1 || normSpeed < 1) {
            throw std::invalid_argument("Requested speed is higher than wheel's max speed");
        }

        /* Compute requested speed in [-MAX_PWM, MAX_PWM] range */
        this->requestedSpeed = this->normAngularToPWM(normSpeed);

        /* Get current time and compute elapsed time since last call of this method */
        unsigned long time = micros();
        float deltaTime = (time-lastUpdateTime) * MICROS;

        /* Compute and update actual speed and store how many steps the wheel turned since last call of this method */
        int steps = this->updateActualSpeed(deltaTime);

        /* Send to the driver the PWM value computed by PID */
        driver.setSpeed(this->updatePID(deltaTime));

        /* Update lastTime with the current one and return the number of radians the wheel turned */
        this->lastUpdateTime = time;
        return this->stepsToRadians * steps;
    }

    /**
     * Number of steps for each encoder complete rotation
     */
    static const int stepsPerEncoderRevolution = 64;

    /**
     * Number of encoder complete rotations for each wheel complete rotation;
     * this number is equal to the gear ratio of the motor
     */
    static const int motorGearRatio = 30;

    /**
     * Radians for each encoder step
     */
    static const float stepsToRadians = TWO_PI / (stepsPerEncoderRevolution * motorGearRatio);

protected:
    /**
     * This method sets the speed of the wheel to maximum value and computes the actual speed in rad/s; it is important
     * to call this method multiple times and for each wheel: keep calling testMaxSpeed() for each wheel for a few
     * seconds, in order to be sure that the wheel reached asymptotic speed; at the end of this process call
     * getTestMaxSpeed(): this will return the maximum speed reached so far; a feasible value of maxSpeed, common for
     * all the wheels, is the minimum speed between the wheels, thus the minimum of the maximums.
     */
    void testMaxSpeed() {
        /* Get current time and compute elapsed time since last call of this method */
        unsigned long time = micros();
        float deltaTime = (time-lastUpdateTime) * MICROS;

        /* Compute and update actual speed */
        this->updateActualSpeed(deltaTime);

        /* Keep maximum between maxSpeed and actualSpeed */
        if(this->actualSpeed > this->maxSpeed) {
            this->maxSpeed = this->actualSpeed;
        }
        /* Make wheel turn at full speed, corresponding to highest PWM value */
        driver.setSpeed(MotorDriver::MAX_PWM);

        /* Update lastTime with the current one */
        this->lastUpdateTime = time;
    }

    /**
     * This method returns the maximum speed reached by calling testMaxSpeed() method; a feasible value of maxSpeed,
     * common for all the wheels, is the minimum speed between the wheels, thus the minimum of the maximums.
     * @return maximum speed found with testMaxSpeed() method in radians per second
     */
    float getTestMaxSpeed() {
        return this->maxSpeed;
    }

    /**
     * This method sets the maximum speed reachable by the wheel; maxSpeed should be found with testMaxSpeed() method;
     * the minimum speed between all the wheels is set after Wheel initialisation, before starting performing movements
     * @param maxSpeed  maximum angular speed of the wheel in radians per second
     */
    void setMaxSpeed(float maxSpeed) {
        this->maxSpeed = maxSpeed;
    }

private:
    /**
     * Motor driver
     */
    const MotorDriver& driver;

    /**
     * Wheel encoder
     */
    const Encoder& encoder;

    /**
     * Maximum angular speed in radians per second
     */
    float maxSpeed;

    /**
     * PID constants
     */
    float kP, kI, kD;

    /**
     * Time PID loop function was last called, expressed in microseconds
     */
    unsigned long lastUpdateTime;

    /**
     * Latest position read from encoder
     */
    int lastEncoderValue = 0;

    /**
     * Last speed requested to the wheel; value in range [-MAX_PWM, MAX_PWM]
     */
    int requestedSpeed = MotorDriver::STILL_PWM;

    /**
     * Actual speed of the wheel in rad/s
     */
    float actualSpeed;

    /**
     * Last error on requested speed
     */
    float lastError = 0;

    /**
     * Cumulative error on requested speed
     */
    float cumulativeError = 0;

    /**
     * This method computes the theoretical PWM value from an angular speed; since this method is used for computing
     * actual speed value to be subtracted to the expected one, absolute value of the return can be greater than MAX_PWM
     * @param angular   speed in rad/s
     * @return PWM value corresponding to the given angular speed; this is usually in range [-MAX_PWM, MAX_PWM]
     */
    int angularToPWM(float angular) {
        /* If maxSpeed is 0, return 0 if also angular is 0, otherwise MAX_PWM with the sign of angular */
        if(maxSpeed == 0) {
            if(angular > 0) {
                return MotorDriver::MAX_PWM;
            }
            else if(angular < 0) {
                return -MotorDriver::MAX_PWM
            }
            else {
                return 0;
            }
        }

        /* Compute and return rounded conversion from angular speed to its corresponding PWM value */
        return (int) ((angular * MotorDriver::MAX_PWM / this->maxSpeed) + 0.5);
    }

    /**
     * This method computes the theoretical PWM value from a normalized angular speed; since this method is used for
     * computing requested speed value
     * @param nAngular  normalized angular speed; number in range [-1, 1]
     * @return PWM value corresponding to the given normalized angular speed; this will be in range [-MAX_PWM, MAX_PWM]
     */
    int normAngularToPWM(float nAngular) {
        /* Compute and return rounded conversion from [-1, 1] to [-MAX_PWM, MAX_PWM] */
        int pwm = (nAngular * MotorDriver::MAX_PWM) + 0.5;
        return constrain(pwm, -MotorDriver::MAX_PWM, MotorDriver::MAX_PWM);
    }

    /**
     * This method computes and stores actual angular speed of the wheel and returns the number of steps performed
     * @param deltaTime time elapsed in seconds since last execution of this method
     * @return number of steps performed by the wheel since last call of this method
     */
    int updateActualSpeed(float deltaTime) {
        /* Read position of the wheel from the encoder and compute the difference from last position */
        int encoderValue = encoder.read();
        int deltaSteps = encoderValue - lastEncoderValue;

        /* Compute and store actual angular speed of the wheel */
        this->actualSpeed = this->stepsToRadians * deltaSteps / deltaTime;

        /* Update last position of the wheel and return the difference between current and last position */
        this->lastEncoderValue = encoderValue;
        return deltaSteps;
    }

    /**
     * This method computes with PID the actual value to be sent to the motor, in order to make it spin at given speed
     * @param deltaTime time elapsed in seconds since last execution of this method
     * @return PWM value to be sent to the driver; it will be in range [-MAX_PWM, MAX_PWM]
     */
    int updatePID(float deltaTime) {
        /* Compute error as the difference between requested and actual speed */
        int error = this->requestedSpeed - this->angularToPWM(this->actualSpeed);

        /* Compute integral of error and update cumulative error */
        this->cumulativeError += error * deltaTime;

        /* Compute output as the weighted sum of proportional, integrative and derivative errors */
        int output = (kP * error) + (kI * cumulativeError) + (kD * (error-lastError)/deltaTime);

        /* Update last error for computing next iteration's derivative error */
        this->lastError = error;

        /* Return output, constrained so that it's in range [-MAX_PWM, MAX_PWM] */
        return constrain(output, -MotorDriver::MAX_PWM, MotorDriver::MAX_PWM);
    }

};

#endif
