#ifndef OMNI3_MR001004_H
#define OMNI3_MR001004_H

#include "../motor_driver.h"

/**
 * Class for handling drivers with 1 PWM and 2 digital inputs
 */
class MR001004: public MotorDriver {
public:
    /**
     * Constructor of MR001004 class
     * @param PWM   Pin number for speed magnitude
     * @param A     Pin number for digital pin A (direction)
     * @param B     Pin number for digital pin B (direction)
     */
    MR001004(unsigned char PWM, unsigned char A, unsigned char B) : PWM(PWM), A(A), B(B) {
        /* Initialize pins as outputs */
        pinMode(PWM, OUTPUT);
        pinMode(A, OUTPUT);
        pinMode(B, OUTPUT);

        /* Set motor speed to 0 */
        this->setSpeed(0);
    }

private:
    /**
     * Pins numbers
     */
    const unsigned char PWM, A, B;

    /**
     * Sets motor speed absolute value
     * @param speed     integer in [0, MAX_PWM] range
     */
    void setMagnitude(int speed) override {
        /* Write a pulse with the given duty cycle */
        analogWrite(this->PWM, speed);
    }

    /**
     * Sets motor direction
     * @param dir       enum indicating whether the motor turns forwards or backwards or if it stays braked or released
     */
    void setDirection(Direction dir) override {
        switch(dir) {
            /* If direction is released, write LOW on both A and B pins */
            case Direction::RELEASED:
                digitalWrite(A, 0);
                digitalWrite(B, 0);
                break;

            /* If direction is forwards, write HIGH on pin A and LOW on pin B */
            case Direction::FORWARDS:
                digitalWrite(A, 1);
                digitalWrite(B, 0);
                break;

            /* If direction is backwards, write HIGH on pin B and LOW on pin A */
            case Direction::BACKWARDS:
                digitalWrite(A, 0);
                digitalWrite(B, 1);
                break;

            /* If direction is braked, write HIGH on both A and B pins */
            case Direction::BRAKED:
                digitalWrite(A, 1);
                digitalWrite(B, 1);
                break;
        }

    }
};

#endif //OMNI3_MR001004_H
