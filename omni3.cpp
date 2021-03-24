#include "omni3.h"

/* public methods */
omni3_params_t Omni3::readStoredData(int memAddr) {
    omni3_params_t data;
    EEPROM.get(memAddr, data);
    return data;
}

bool Omni3::home() {
    /* If currentSpeed is not { 0.0, 0.0, 0.0 }, return false */
    if (this->displacement[FORWARD] != 0.0 ||
        this->displacement[STRAFE] != 0.0 ||
        this->displacement[THETA] != 0.0 )
    {
        return false;
    }

    /* Otherwise (the robot is still), set current and target positions to { 0.0, 0.0, 0.0 } and return true */
    for (int i=0; i < DOF; i++) {
        this->currentPosition[i] = 0.0;
        this->targetPosition[i] = 0.0;
    }
    return true;
}

/* private methods */
void Omni3::setWheelsRadius (double wheelsRadius) {
    this->R = wheelsRadius;
    this->C30_R = COS30 / wheelsRadius;
    this->C60_R = COS60 / wheelsRadius;
    this->C180_R = COS180 / wheelsRadius;
    this->T30R = TAN30 * wheelsRadius;
    this->R_3 = wheelsRadius / 3;
    this->L_R = this->L / wheelsRadius;
    this->R_3L = wheelsRadius / (3*this->L);
}

void Omni3::setRobotRadius (double robotRadius) {
    this->L = robotRadius;
    this->L_R = robotRadius / this->R;
    this->R_3L = this->R / (3*robotRadius);
}

void Omni3::directKinematics(const double* angularDisplacement) {
    /* forward = tan(30°) * R * (wR - wL) */
    this->displacement[FORWARD] = T30R * (angularDisplacement[W_RIGHT] - angularDisplacement[W_LEFT]);

    /* strafe = R/3 * (wR - 2*wB + wL) */
    this->displacement[STRAFE] = R_3 * (angularDisplacement[W_RIGHT] - 2*angularDisplacement[1] + angularDisplacement[2]);

    /* theta = R/(3*L) * (wR + wB + wL) */
    this->displacement[THETA] = R_3L * (angularDisplacement[0] + angularDisplacement[1] + angularDisplacement[2]);
}

void Omni3::inverseKinematics(const double* speed) const {
    const double S = C60_R * speed[STRAFE];
    const double F = C30_R * speed[FORWARD];
    const double T = L_R * speed[THETA];

    /* wR = cos(60°)/R * strafe + cos(30°)/R * forward - L/R * theta */
    wheels[W_RIGHT]->setSpeed(S + F - T);

    /* wB = cos(180°)/R * strafe - L/R * theta */
    wheels[W_BACK]->setSpeed(C180_R * speed[STRAFE] - T);

    /* wL = cos(60°)/R * strafe - cos(30°)/R * forward - L/R * theta */
    wheels[W_LEFT]->setSpeed(S - F - T);
}

void Omni3::odometry() {
    const double th = currentPosition[POS_PHI] + displacement[THETA]/2.0;

    this->currentPosition[POS_X] = cos(th)*displacement[POS_X] - sin(th)*displacement[POS_Y];
    this->currentPosition[POS_Y] = sin(th)*displacement[POS_X] + cos(th)*displacement[POS_Y];
    this->currentPosition[POS_PHI] = currentPosition[POS_PHI] + displacement[THETA];
}