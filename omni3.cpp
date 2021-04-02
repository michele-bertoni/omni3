#include "omni3.h"

/* Public methods */
omni3_params_t Omni3::readStoredData(int memAddr) {
    /* initialize data */
    omni3_params_t data;

    /* read EEPROM at given memory address and return read data */
    EEPROM.get(memAddr, data);
    return data;
}

void Omni3::handle() {
    /* Read current time */
    unsigned long time = millis();

    /* Initialize array with target strafe, forward and angular speeds */
    double targetSpeed[DOF] = {0.0, 0.0, 0.0};

    /* Compute angular displacement of each wheel */
    double angularDisplacement[WHEELS_NUM];
    angularDisplacement[W_RIGHT] = wheels[W_RIGHT]->handle();
    angularDisplacement[W_BACK] = wheels[W_BACK]->handle();
    angularDisplacement[W_LEFT] = wheels[W_LEFT]->handle();

    /* From angular displacements, compute the current position of the robot */
    directKinematics(angularDisplacement);
    odometry();

    /* Compute delta time in seconds */
    double dt = (time - lastTime) * MILLIS;

    /* Initialize array with current strafe, forward and angular speeds */
    double currentSpeed[DOF];
    currentSpeed[FORWARD] = displacement[FORWARD] / dt;
    currentSpeed[STRAFE] = displacement[STRAFE] / dt;
    currentSpeed[THETA] = displacement[THETA] / dt;

    /* Compute target speed vector */
    bool isNormalized = this->movementsHandler.handle(currentPosition, currentSpeed, time, targetSpeed);

    /* Update lastTime with current time */
    lastTime = time;

    /* Compute inverse kinematics, requesting speeds to the motors: if some fails, emergency stop the robot */
    if (isNormalized ? !normalizedInverseKinematics(targetSpeed) : !inverseKinematics(targetSpeed)) {
        this->emergencyStop();
        return;
    }
}

bool Omni3::home() {
    /* If currentSpeed is not { 0.0, 0.0, 0.0 }, return false */
    if (this->displacement[FORWARD] != 0.0 ||
        this->displacement[STRAFE] != 0.0 ||
        this->displacement[THETA] != 0.0 )
    {
        return false;
    }

    /* Otherwise (the robot is still), set current position to { 0.0, 0.0, 0.0 } and return true */
    for (double & i : this->currentPosition) {
        i = 0.0;
    }
    return true;
}

void Omni3::emergencyStop() {
    /* Set max speed to every wheel to 0 */
    for (auto & wheel : wheels) {
        wheel->setMaxSpeed(0.0);
    }
}

bool Omni3::handleMessage(byte message, double *args) {
    uint8_t argsLen = message & 0b00000111;
    byte msgType = message >> 3;
    /* Movements */
    if (msgType >= 0b10000) {
        return handleMovementsMessage(msgType & 0b1111, argsLen, args);
    }
    else {
        /* Testers and setters */
        if (msgType >= 0b01000) {
            if(argsLen==0) {
                return handleTestersMessage(msgType & 0b111);
            }
            else {
                return handleSettersMessage(msgType & 0b111, argsLen, args);
            }
        }
        /* Functions */
        else {
            return handleFunctionsMessage(msgType & 0b111);
        }
    }
}

void Omni3::setMaxWheelSpeed(double speed) {
    /* For each wheel, set max speed */
    for (auto & wheel : wheels) {
        wheel->setMaxSpeed(speed);
    }
}

void Omni3::setWheelsRadius (double wheelsRadius) {
    /* Set various constants */
    this->R = wheelsRadius;
    this->C30_R = COS30 / wheelsRadius;
    this->S30_R = SIN30 / wheelsRadius;
    this->C180_R = COS180 / wheelsRadius;
    this->T30R = TAN30 * wheelsRadius;
    this->R_3 = wheelsRadius / 3;
    this->L_R = this->L / wheelsRadius;
    this->R_3L = wheelsRadius / (3*this->L);
}

void Omni3::setRobotRadius (double robotRadius) {
    /* Set various constants */
    this->L = robotRadius;
    this->L_R = robotRadius / this->R;
    this->R_3L = this->R / (3*robotRadius);
}

void Omni3::setPIDConstants(double kP, double kI, double kD) {
    /* For each wheel, set PID constants */
    for (auto & wheel : wheels) {
        wheel->setPID(kP, kI, kD);
    }
}

/* Private methods */
void Omni3::directKinematics(const double* angularDisplacement) {
    /* forward = tan(30°)*R * (wR - wL) */
    this->displacement[FORWARD] = T30R *
            (angularDisplacement[W_RIGHT] - angularDisplacement[W_LEFT]);

    /* strafe = R/3 * (wR - 2*wB + wL) */
    this->displacement[STRAFE] = R_3 *
            (angularDisplacement[W_RIGHT] - 2*angularDisplacement[W_BACK] + angularDisplacement[W_LEFT]);

    /* theta = R/(3*L) * (wR + wB + wL) */
    this->displacement[THETA] = R_3L *
            (angularDisplacement[W_RIGHT] + angularDisplacement[W_BACK] + angularDisplacement[W_LEFT]);
}

bool Omni3::inverseKinematics(const double* speed) const {
    /* Compute S, F and T components */
    const double S = S30_R * speed[STRAFE];
    const double F = C30_R * speed[FORWARD];
    const double T = L_R * speed[THETA];

    /* wR = sin(30°)/R * strafe + cos(30°)/R * forward + L/R * theta */
    return wheels[W_RIGHT]->setSpeed(S + F + T) &&

    /* wB = cos(180°)/R * strafe + L/R * theta */
    wheels[W_BACK]->setSpeed(C180_R*speed[STRAFE] + T) &&

    /* wL = sin(30°)/R * strafe - cos(30°)/R * forward + L/R * theta */
    wheels[W_LEFT]->setSpeed(S - F + T);
}

bool Omni3::normalizedInverseKinematics(const double* speed) const {
    /* Compute S, F and T components */
    const double S = SIN30 * speed[STRAFE];
    const double F = COS30 * speed[FORWARD];
    const double T = speed[THETA];

    /* wR = sin(30°)*strafe + cos(30°)*forward + theta */
    return wheels[W_RIGHT]->setNormalizedSpeed(S + F + T) &&

    /* wB = cos(180°)*strafe + theta */
    wheels[W_BACK]->setNormalizedSpeed(COS180*speed[STRAFE] + T) &&

    /* wL = sin(30°)*strafe - cos(30°)*forward + theta */
    wheels[W_LEFT]->setNormalizedSpeed(S - F + T);
}

void Omni3::odometry() {
    /* Compute average angle during last movement */
    const double alpha = currentPosition[POS_PHI] + displacement[THETA] / 2.0;

    /* x' = x*cos(alpha) - y*sin(alpha) */
    this->currentPosition[POS_X] = cos(alpha) * displacement[POS_X] - sin(alpha) * displacement[POS_Y];

    /* y' = x*sin(alpha) + y*cos(alpha) */
    this->currentPosition[POS_Y] = sin(alpha) * displacement[POS_X] + cos(alpha) * displacement[POS_Y];

    /* phi' = phi + theta, phi' in [0, 2*PI) */
    this->currentPosition[POS_PHI] = currentPosition[POS_PHI] + displacement[THETA];
    while (this->currentPosition[POS_PHI] >= TWO_PI) {
        this->currentPosition[POS_PHI] -= TWO_PI;
    }
    while (this->currentPosition[POS_PHI] < 0.0) {
        this->currentPosition[POS_PHI] += TWO_PI;
    }
}

bool Omni3::handleMovementsMessage(uint8_t movementType, uint8_t argsLen, double *args) {
    switch(movementType) {
        case 0:
            if (argsLen==0) {
                movementsHandler.addStop();
                return true;
            }
            break;
        case 1:
            if (argsLen==3) {
                movementsHandler.addConstantSpeedMovement(args[0], args[1], args[2]);
                return true;
            }
            break;
        case 2:
            if (argsLen==3) {
                movementsHandler.addConstantNormSpeedMovement(args[0], args[1], args[2]);
                return true;
            }
            break;
        case 3:
            if (argsLen==4) {
                return movementsHandler.addTargetPosTime(args[0], args[1], args[2], args[3]);
            }
            break;
        case 4:
            if (argsLen==5) {
                return movementsHandler.addTargetPosSpeed(
                        args[0], args[1], args[2], args[3], args[4]);
            }
            break;
        case 5:
            if (argsLen==5) {
                return movementsHandler.addTargetPosNormSpeed(
                        args[0], args[1], args[2], args[3], args[4]);
            }
            break;
        case 6:
            if (argsLen==4) {
                return movementsHandler.addTargetSpeedTime(
                        args[0], args[1], args[2], args[3]);
            }
            break;
        case 7:
            if (argsLen==4) {
                return movementsHandler.addTargetNormSpeedTime(
                        args[0], args[1], args[2], args[3]);
            }
            break;
        default:
            return false;
    }
    return false;
}

bool Omni3::handleTestersMessage(uint8_t testType) {
    switch (testType) {
        case 0:

            break;
        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            break;
        default:
            return false;
    }
    return false;
}

bool Omni3::handleSettersMessage(uint8_t setterType, uint8_t argsLen, double *args) {
    return false;
}

bool Omni3::handleFunctionsMessage(uint8_t functionType) {
    return false;
}
