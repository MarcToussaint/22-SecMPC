#include "experiment.h"

#include <KOMO/manipTools.h>

void testPnp() {
  rai::Configuration C;
  C.addFile("graspScenario.g");
  arr qHome = C.getJointState();

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(4, 1, 3., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();

  // homing
  if(qHome.N) komo.addObjective({}, FS_qItself, {}, OT_sos, {.1}, qHome);

  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

  const char* gripperName="l_gripper";
  const char* palmName="l_palm";
  const char* boxName="box";
  const char* targetName="target";
  //const char* arm1Name="l_panda_coll7";
  //const char* arm2Name="l_panda_coll6";
  arr boxSize={.07,.08,.06};
  rai::Enum<rai::ArgWord> pickDirection = rai::_xAxis;
  rai::Enum<rai::ArgWord> placeDirection = rai::_zAxis;

  //-- pick
  //pregrasp
  addBoxPickObjectives(komo, 1., pickDirection, boxName, boxSize, gripperName, palmName, targetName, true);

  //grasp
  komo.addModeSwitch({2.,4.}, rai::SY_stable, {gripperName, boxName}, true);
  addBoxPickObjectives(komo, 2., pickDirection, boxName, boxSize, gripperName, palmName, targetName);

  //lift
  //komo.addObjective({3.}, FS_distance, {boxName, "table"}, OT_ineq, {1e1}, {-.1});

  //-- place
  //pre
  addBoxPlaceObjectives(komo, 3., placeDirection, boxName, boxSize, targetName, gripperName, palmName, -.02, true);
  //place
  komo.addModeSwitch({4.,-1.}, rai::SY_stable, {"table", boxName}, false);
  addBoxPlaceObjectives(komo, 4., placeDirection, boxName, boxSize, targetName, gripperName, palmName);

#if 0 //only for development
  komo.optimize();
  cout <<komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
#endif


  SecMPC_Experiments ex(C, komo, .1, 1e0, 0.5);
  ex.selectSubSeq(0, 1);

  if(ex.bot && ex.bot->gripperL){ ex.bot->gripperL->open(); while(!ex.bot->gripperL->isDone()) rai::wait(.1); }

  arr boxCen, boxVel;

  while(ex.step()){
    if(ex.bot && ex.bot->optitrack){
      C[boxName]->setPose(C["green3"]->getPose());
//      C[boxName]->setPosition(C["b2"]->getPosition());
    }else{
      //randomWalkPosition(C[boxName], boxCen, boxVel, .001);
    }
  }

  if(ex.bot && ex.bot->gripperL){ ex.bot->gripperL->close(1.5); while(!ex.bot->gripperL->isDone()) rai::wait(.1); }
  C.attach(gripperName, boxName);
//  rai::wait();

  ex.selectSubSeq(2,-1);

  while(ex.step()){
    if(ex.bot && ex.bot->optitrack){
      C[targetName]->setPosition(C["b1"]->getPosition());
    }
  }

  if(ex.bot && ex.bot->gripperL){ ex.bot->gripperL->open(); while(!ex.bot->gripperL->isDone()) rai::wait(.1); }
  C.attach(targetName, boxName);
//  rai::wait();

  ex.bot->home(ex.C);
//  rai::wait();

}
