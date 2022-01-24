#include "experiment.h"

void testBallFollowing() {
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  C.addFrame("HandStick", "optitrack_base") ->setShape(rai::ST_marker, {.1});

  C.addFrame("ball", "table")
      ->setShape(rai::ST_sphere, {.03})
      .setColor({1.,0,0})
      .setRelativePosition(arr{-.4,.4,.4});

  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(1., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();

  komo.addObjective({1.}, FS_positionDiff, {"l_gripper", "ball"}, OT_eq, {1e1});

#if 0 //only for development
  komo.optimize();
  komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
#endif

  SecMPC_Experiments ex(C, komo, .02, 1e0, 1e0);

  bool useSimulatedBall=!rai::getParameter<bool>("bot/useOptitrack", false);
  arr ballVel, ballCen;

  while(ex.step()){
    if(useSimulatedBall){
      randomWalkPosition(C["ball"], ballCen, ballVel, .001);
    }else{
      C["ball"]->setPosition(C["HandStick"]->getPosition());
    }
  }
}
