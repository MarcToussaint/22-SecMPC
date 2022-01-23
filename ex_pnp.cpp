#include "experiment.h"

#include <KOMO/manipTools.h>

void testPnp() {
  rai::Configuration C;
  C.addFile("graspScenario.g");
  arr qHome = C.getJointState();

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(3, 1, 3., 1);
  komo.add_qControlObjective({}, 1, 1e-1);

  // homing
  if(qHome.N) komo.addObjective({}, FS_qItself, {}, OT_sos, {.1}, qHome);

  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1e1});

  const char* gripperName="l_gripper";
  const char* palmName="l_palm";
  const char* boxName="box";
  const char* targetName="target";
  //const char* arm1Name="l_panda_coll7";
  //const char* arm2Name="l_panda_coll6";
  arr boxSize={.06,.15,.09};
  rai::Enum<rai::ArgWord> pickDirection = rai::_xAxis;
  rai::Enum<rai::ArgWord> placeDirection = rai::_zAxis;

  //-- pick
  addBoxPickObjectives(komo, 1., pickDirection, boxName, boxSize, gripperName, palmName, targetName, true);
  komo.addModeSwitch({2.,3.}, rai::SY_stable, {gripperName, boxName}, true);
  addBoxPickObjectives(komo, 2., pickDirection, boxName, boxSize, gripperName, palmName, targetName);

  //-- place
  komo.addModeSwitch({3.,-1.}, rai::SY_stable, {"table", boxName}, false);
  addBoxPlaceObjectives(komo, 3., placeDirection, boxName, boxSize, targetName, gripperName, palmName);

#if 1 //only for development
  komo.optimize();
  cout <<komo.getReport(true);
  komo.view(true, "optimized motion");
  while(komo.view_play(true));
#endif

  SecMPC_Experiments ex(C, komo, .1, 1e0, 1e0);
  while(ex.step(komo.objectives));
}
