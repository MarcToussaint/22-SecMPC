#include "experiment.h"

void testBallReaching() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  C.addFrame("ball", "table")
      ->setShape(rai::ST_sphere, {.03})
      .setColor({1.,0,0})
      .setRelativePosition(arr{-.4,.4,.4});

  //-- define constraints
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(2., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();
  komo.addObjective({1.}, FS_positionRel, {"ball", "l_gripper"}, OT_eq, {6e1}, {0., 0., -.1});
  komo.addObjective({1., 2.}, FS_positionRel, {"ball", "l_gripper"}, OT_eq, {{2,3},{1e1,0,0,0,1e1,0}});
  komo.addObjective({2.}, FS_positionDiff, {"l_gripper", "ball"}, OT_eq, {6e1});

  SecMPC_Experiments ex(C, komo, .1, 1e0, 1e0);
  ex.step();
  ex.mpc->timingMPC.backtrackingTable=uintA{0, 0};

  bool useSimulatedBall=!rai::getParameter<bool>("bot/useOptitrack", false);
  arr ballVel = {.0, .0, .0};
  arr ballCen = C["ball"]->getPosition();

  while(ex.step()){
    if(useSimulatedBall){
      if(!(ex.stepCount%20)){
        ballVel(0) = .01 * rnd.gauss();
        ballVel(2) = .01 * rnd.gauss();
      }
      if(!(ex.stepCount%40)) ballVel=0.;
      randomWalkPosition(C["ball"], ballCen, ballVel);
    }else{
      C["ball"]->setPosition(C["HandStick"]->getPosition());
    }
  }
}
