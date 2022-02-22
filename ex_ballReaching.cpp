#include "experiment.h"

void testBallReaching() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  C.addFrame("HandStick", "optitrack_base") ->setShape(rai::ST_marker, {.1});

  C.addFrame("ball", "table")
      ->setShape(rai::ST_sphere, {.03})
      .setColor({1.,0,0})
      .setRelativePosition(arr{-.4,.4,.4});

  C.addFrame("obst", "table")
      ->setShape(rai::ST_capsule, {.1, .08})
      .setColor({.9})
      .setRelativePosition(arr{-.0,.4,.4});

  //-- define constraints
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(2., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();
  komo.addObjective({1.}, FS_positionRel, {"ball", "l_gripper"}, OT_eq, {1e1}, {0., 0., -.07});
  komo.addObjective({1., 2.}, FS_positionRel, {"ball", "l_gripper"}, OT_eq, {{2,3},{1e1,0,0,0,1e1,0}});
  komo.addObjective({2.}, FS_positionDiff, {"l_gripper", "ball"}, OT_eq, {1e1});

  SecMPC_Experiments ex(C, komo, .02, 1e0, 1e0);
  ex.step();
  ex.mpc->tauCutoff = .1;
  ex.mpc->timingMPC.backtrackingTable=uintA{0, 0, 0};
  //ex.mpc->verbose = 3;

  bool useSimulatedBall=!rai::getParameter<bool>("bot/useOptitrack", false);
  arr ballVel, ballCen;

  while(ex.step()){
    if(useSimulatedBall){
      randomWalkPosition(C["ball"], ballCen, ballVel, .0005);
    }else{
      C["ball"]->setPosition(C["HandStick"]->getPosition());
    }
  }
}
