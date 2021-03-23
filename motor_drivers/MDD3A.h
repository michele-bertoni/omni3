#ifndef OMNI3_MDD3A_H
#define OMNI3_MDD3A_H

#include "../motor_driver.h"

/**
 * Class for handling drivers with 2 PWM inputs
 */
class MDD3A: public MotorDriver {
public:
    /**
     * Constructor of MR001004 class
     * @param A     Pin number for analog pin A
     * @param B     Pin number for analog pin B
     */
    MDD3A(uint8_t A, uint8_t B) : A(A), B(B) {
        /* Initialize pins as outputs */
        pinMode(this->A, OUTPUT);
        pinMode(this->B, OUTPUT);

        /* Initialize isPinHigh */
        this->isAHigh = false;
        this->isBHigh = false;

        /* Set motor speed to 0 */
        this->setSpeed(0);
    }

private:
    /**
     * Pins numbers
     */
    const uint8_t A, B;

    /**
     * Booleans used for storing direction information
     */
    bool isAHigh, isBHigh;

    /**
     * Sets motor speed absolute value
     * @param speed     integer in [0, MAX_PWM] range
     */
    void setMagnitude(int speed) override {
        /* Write a pulse with the given duty cycle multiplied by the direction */
        analogWrite(this->isAHigh*this->A, speed);
        analogWrite(this->isBHigh*this->B, speed);
    }

    /**
     * Sets motor direction
     * @param dir       enum indicating whether the motor turns forwards or backwards or if it stays braked or released
     */
    void setDirection(Direction dir) override {
        switch(dir) {
            /* If direction is released, store LOW on both A and B pins */
            case Direction::RELEASED:
                this->isAHigh = false;
                this->isBHigh = false;
                break;

            /* If direction is forwards, store HIGH on pin A and LOW on pin B */
            case Direction::FORWARDS:
                this->isAHigh = true;
                this->isBHigh = false;
                break;

            /* If direction is backwards, store HIGH on pin B and LOW on pin A */
            case Direction::BACKWARDS:
                this->isAHigh = false;
                this->isBHigh = true;
                break;

            /* If direction is braked, store HIGH on both A and B pins */
            case Direction::BRAKED:
                this->isAHigh = true;
                this->isBHigh = true;
                break;
        }

    }
};


#endif //OMNI3_MDD3A_H
