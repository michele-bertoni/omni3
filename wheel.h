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
     * @param radius    Radius of the wheel in meters
     * @param maxSpeed  Maximum angular speed of the wheel in rad/s
     */
    Wheel(const MotorDriver& driver, const Encoder& encoder, float radius, float maxSpeed) :
            driver(driver), encoder(encoder), radius(radius), maxSpeed(maxSpeed) {
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
     * @return radians the wheel rotated since last call of this function
     */
    float setSpeed(float speed) {
        return this->setNormalizedSpeed(speed / this->maxSpeed);
    }

    /**
     * This method sets the target speed of the motor and returns how many steps it moved from last time it was called
     * @param normSpeed requested speed in range [-1, 1]
     * @return radians the wheel rotated since last call of this function
     */
    float setNormalizedSpeed(float normSpeed) {
        this->requestedSpeed = this->normAngularToPWM(normSpeed);
        unsigned long time = micros();
        float deltaTime = (time-lastUpdateTime) * MICROS;
        int steps = this->updateActualSpeed(deltaTime);
        driver.setSpeed(this->updatePID(deltaTime));
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
     * Radius of the wheel in meters
     */
    const float radius;

    /**
     * Maximum angular speed in radians per second
     */
    const float maxSpeed;

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
     * This method computes the theoretical PWM value from an angular speed
     * @param angular   speed in rad/s
     * @return PWM value corresponding to the given angular speed
     */
    int angularToPWM(float angular) {
        return (int) ((angular * MotorDriver::MAX_PWM / this->maxSpeed) + 0.5);
    }

    /**
     * This method computes the theoretical PWM value from a normalized angular speed
     * @param nAngular  normalized angular speed; number in range [-1, 1]
     * @return PWM value corresponding to the given normalized angular speed; this will be in range [-MAX_PWM, MAX_PWM]
     */
    int normAngularToPWM(float nAngular) {
        int pwm = (nAngular * MotorDriver::MAX_PWM) + 0.5;
        return constrain(pwm, -MotorDriver::MAX_PWM, MotorDriver::MAX_PWM);
    }

    /**
     * This method computes and stores actual angular speed of the wheel and returns the number of steps performed
     * @param deltaTime time elapsed in seconds since last execution of this method
     * @return number of steps performed by the wheel since last call of this method
     */
    int updateActualSpeed(float deltaTime) {
        int encoderValue = encoder.read();
        int deltaSteps = encoderValue - lastEncoderValue;
        this->actualSpeed = this->stepsToRadians * deltaSteps / deltaTime;
        this->lastEncoderValue = encoderValue;
        return deltaSteps;
    }

    /**
     * This method computes with PID the actual value to be sent to the motor, in order to make it spin at given speed
     * @param deltaTime time elapsed in seconds since last execution of this method
     * @return PWM value to be sent to the driver; it will be in range [-MAX_PWM, MAX_PWM]
     */
    int updatePID(float deltaTime) {
        int error = this->requestedSpeed - this->angularToPWM(this->actualSpeed);
        this->cumulativeError += error * deltaTime;
        int output = (kP * error) + (kI * cumulativeError) + (kD * (error-lastError)/deltaTime);
        this->lastError = error;
        return constrain(output, -MotorDriver::MAX_PWM, MotorDriver::MAX_PWM);
    }

};

#endif //OMNI3_WHEEL_H
