//
// Created by atabak on 26-10-2018.
//

#ifndef DRAKE_CMAKE_INSTALLED_LQR_ACROBOT_H
#define DRAKE_CMAKE_INSTALLED_LQR_ACROBOT_H


// Eigen, used by drake
#include "drake/systems/controllers/linear_quadratic_regulator.h"


// System dynamics matrices: https://github.com/RobotLocomotion/drake/blob/master/systems/controllers/zmp_planner.h#L306
drake::systems::controllers::LinearQuadraticRegulatorResult lqr_result;
drake::systems::controllers::LinearQuadraticRegulatorResult getLQRControl();


#endif //DRAKE_CMAKE_INSTALLED_LQR_ACROBOT_H
