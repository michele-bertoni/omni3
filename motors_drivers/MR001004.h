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
    MR001004(unsigned char PWM, unsigned char A, unsigned char B) {
        /* Set pins numbers */
        this->_PWM = PWM;
        this->_A = A;
        this->_B = B;

        /* Initialize pins as outputs */
        pinMode(_PWM, OUTPUT);
        pinMode(_A, OUTPUT);
        pinMode(_B, OUTPUT);

        /* Set motor speed to 0 */
        this->setSpeed(0);
    }

private:
    /**
     * Pins numbers
     */
    unsigned char _PWM, _A, _B;

    /**
     * Sets motor speed absolute value
     * @param speed     integer in [0, MAX_PWM] range
     */
    void _setMagnitude(int speed) {
        /* Write a pulse with the given duty cycle */
        analogWrite(this->_PWM, speed);
    }

    /**
     * Sets motor direction
     * @param dir       enum indicating whether the motor turns forwards or backwards or if it stays braked or released
     */
    void _setDirection(Direction dir) {
        switch(dir) {
            /* If direction is released, write LOW on both A and B pins */
            case Direction::RELEASED:
                digitalWrite(this->_A, LOW);
                digitalWrite(this->_B, LOW);
                break;

            /* If direction is forwards, write HIGH on pin A and LOW on pin B */
            case Direction::FORWARDS:
                digitalWrite(this->_A, HIGH);
                digitalWrite(this->_B, LOW);
                break;

            /* If direction is backwards, write HIGH on pin B and LOW on pin A */
            case Direction::FORWARDS:
                digitalWrite(this->_A, LOW);
                digitalWrite(this->_B, HIGH);
                break;

            /* If direction is braked, write HIGH on both A and B pins */
            case Direction::BRAKED:
                digitalWrite(this->_A, HIGH);
                digitalWrite(this->_B, HIGH);
                break;
        }

    }
};

#endif //OMNI3_MR001004_H
