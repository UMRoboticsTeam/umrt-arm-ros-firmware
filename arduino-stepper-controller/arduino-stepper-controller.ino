#include "ConnectionManager.hpp"
#include <MobaTools.h>

#define STEP_1 3
#define DIR_1 2
#define CS_1 4
#define CURRENT_1 2000
#define FULLROT_1 200

ConnectionManager manager;
MoToStepper step_1(FULLROT_1, STEPDIR);

void setup() {
  // put your setup code here, to run once:
  ConnectionManager::init();

  manager.add_driver(STEP_1, DIR_1, CS_1, CURRENT_1);

  step_1.attach(STEP_1, DIR_1);

  // =========TESTING=========
  step_1.setSpeed(10000); // 1000 RPM
  step_1.rotate(1);
  // =========================
}

void loop() {
  // put your main code here, to run repeatedly:

}
