#include "omni3.h"

Omni3 *robot;

void setup() {
    robot = new Omni3(new Wheel(new MDD3A(3, 4), new Encoder(9, 10)),
                      new Wheel(new MDD3A(5, 6), new Encoder(11, 12)),
                      new Wheel(new MDD3A(7, 8), new Encoder(13, 14)),
                      0);
}

void loop() {
    double args[MAX_ARGS];
    robot->handleMessage(0, args);
    robot->handle();

}
