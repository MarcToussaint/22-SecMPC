#include "experiment.h"

#include <Kin/F_forces.h>
#include <KOMO/skeleton.h>
#include <Kin/viewer.h>
#include <Gui/opengl.h>

void testPushing2() {
  rai::Configuration C;
  C.addFile("pushScenario.g");

  const char* pusher="stickTip";
  const char* obj="puck";
  const char* target="target";

  C.addFrame("sph", obj)
      ->setShape(rai::ST_sphere, {.07})
      .setColor({.7,.7,.9,.2})
      .setContact(-1);

  C.watch();

//  C[pusher]->setJoint(rai::JT_free);
  arr targetPos = C[target]->getPosition();
  C[target]->setContact(0);

  rai::Skeleton S = {
    {1., 2.5, rai::SY_quasiStaticOn, {"table", obj} },
    {1., 2., rai::SY_push, {pusher, "sph"} },
//    {2., 2., rai::SY_touch, {obj, target} },
  };
  StringA collisions = {
//    pusher, "sph",
    pusher, obj,
    pusher, "table"
  };

  KOMO komo;
  komo.setModel(C, true);

  S.setKOMO(komo, rai::_sequence);

  komo.addObjective({2., 2.5}, FS_positionDiff, {obj, target}, OT_eq, {1e1});

  for(uint i=0;i<collisions.N;i+=2){
    komo.addObjective({}, FS_distance, {collisions(i), collisions(i+1)}, OT_ineq, {1e1});
  }

  komo.add_collision(true);

#if 1 //for development only
  komo.optimize();
  cout <<komo.getReport(true);
  komo.pathConfig.gl()->ensure_gl().resize(700,500);
  komo.view(true, "solution");
  while(komo.view_play(true, 2., "z.vid/"));
  return;
#endif

  bool useOptitrack=rai::getParameter<bool>("bot/useOptitrack", false);

  SecMPC_Experiments ex(C, komo);
  ex.step();
//  ex.mpc->timingMPC.backtrackingTable=uintA{0, 0, 0, 0, 0};

  while(ex.step()){
    if(useOptitrack){
      C[obj]->setPosition(C["marc_red"]->getPosition());
    }
  }
}
