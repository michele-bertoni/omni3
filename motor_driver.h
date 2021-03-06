#ifndef OMNI3_MOTOR_DRIVER_H
#define OMNI3_MOTOR_DRIVER_H

/**
 * Abstract class for defining motor drivers; virtual methods setSpeed and setDirection must be implemented as described
 * in the methods documentation; a proper constructor receiving output pins must be added as well
 */
class MotorDriver {
public:
    /**
     * Virtual destructor
     */
    virtual ~MotorDriver() = default;

    /**
     * Max PWM value; feasible speed will be in [-MAX_PWM, MAX_PWM] range
     */
    static const int MAX_PWM = 255;

    /**
     * Min PWM value; this is the value sent to the motor in order to keep it still
     */
    static const int STILL_PWM = 0;

    /**
     * Method for setting motor speed
     * @param _speed     integer in [-MAX_PWM, MAX_PWM] range; sign indicates motor's direction of rotation
     */
    void setSpeed(int _speed) {
        /* Force speed in [-MAX_PWM, MAX_PWM] range */
        _speed = MotorDriver::rangedSpeed(_speed);

        /* Update current speed */
        this->speed = _speed;

        /* Compute direction according to speed sign, then extract speed absolute value */
        Direction direction = Direction::RELEASED;
        if(_speed > 0) {
            direction = Direction::FORWARDS;
        }
        if(_speed < 0) {
            direction = Direction::BACKWARDS;
            _speed = -_speed;
        }

        /* Set rotation direction and magnitude */
        this->setDirection(direction);
        this->setMagnitude(_speed);
    }

    /**
     * Returns the current speed of the motor
     * @return current speed of the motor; returned value will be in [-MAX_PWM, MAX_PWM] range
     */
    int getSpeed() const {
        return speed;
    }

protected:
    /**
     * Pure virtual method for setting motor speed; it must be implemented so that speed magnitude of the motor is equal
     * to the requested one
     * @param speed     integer in [0, MAX_PWM] range
     */
    virtual void setMagnitude(int speed) = 0;

    /**
     * Enumeration for describing motor behavior:
     * RELEASED     Motor is free to rotate
     * FORWARDS     Motor rotates forwards at given speed
     * BACKWARDS    Motor rotates backwards at given speed
     * BRAKED       Motor will use engine brake for keeping its position
     */
    enum class Direction {RELEASED, FORWARDS, BACKWARDS, BRAKED};

    /**
     * Pure virtual method for setting motor direction; it must be implemented so that speed direction of the motor is
     * equal to the requested one
     * @param dir       enum indicating whether the motor turns forwards or backwards or if it stays braked or released
     */
    virtual void setDirection(Direction dir) = 0;

private:
    /**
     * Current speed of the motor
     */
    int speed=0;

    /**
     * Returns a speed in the [-MAX_PWM, MAX_PWM] range
     * @param speed     value to be normalized
     * @return ranged speed
     */
    static int rangedSpeed(int speed) {
        /* If speed is higher than MAX, return MAX */
        if(speed > MotorDriver::MAX_PWM) {
            return MotorDriver::MAX_PWM;
        }

        /* If speed is lower than MIN, return MIN */
        if(speed < -MotorDriver::MAX_PWM) {
            return -MotorDriver::MAX_PWM;
        }

        /* Speed was already in the range, just return speed */
        return speed;
    }
};

#endif
