#include "experiment.h"

#include <Kin/F_forces.h>
#include <Kin/viewer.h>
#include <Gui/opengl.h>

void testPushing() {
  rai::Configuration C;
  C.addFile("pushScenario.g");

  const char* pusher="stickTip";
  const char* obj="puck";
  const char* target="target";
  const char* table="table";

  //-- define constraints
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(4., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addObjective({1.}, make_shared<F_PushRadiusPrior>(.13), {pusher, obj, target}, OT_eq, {1e1}, {0., 0., .1});
  komo.addObjective({2.}, make_shared<F_PushRadiusPrior>(.10), {pusher, obj, target}, OT_eq, {1e1}, {0., 0., .02});
  komo.addObjective({3.}, make_shared<F_PushRadiusPrior>(.05), {pusher, obj, target}, OT_eq, {1e1}, {0., 0., .02});
  komo.addObjective({4.}, make_shared<F_PushRadiusPrior>(.05), {pusher, obj, target}, OT_eq, {1e1}, {0., 0., .02});
  komo.addObjective({1., 4.}, make_shared<F_PushAligned>(), {pusher, obj, target}, OT_eq, {{1,3},{0,0,1e1}});
  komo.addObjective({1., 4.}, make_shared<F_PushSide>(), {pusher, obj, target}, OT_ineq, {1e1});

  //low-level kinematic switch
  double switchTime=3.;
  rai::Transformation rel = 0;
  rel.pos.set(0, 0, .5*(shapeSize(komo.world, table) + shapeSize(komo.world, obj)));
  komo.addSwitch({switchTime,-1}, true, make_shared<rai::KinematicSwitch>(rai::SW_joint, rai::JT_transXY, table, obj, komo.world, rai::SWInit_copy, 0, rel, NoTransformation));
  komo.addObjective({switchTime}, FS_pose, {obj}, OT_eq, {1e1}, NoArr, 1);

  komo.addObjective({4.}, FS_positionDiff, {obj, target}, OT_eq, {1e1});

  //komo.addObjective({1., 2.}, FS_positionRel, C, {"ball", "l_gripper"}, OT_eq, {{2,3},{1e1,0,0,0,1e1,0}});
  //komo.addObjective({2.}, FS_positionDiff, C, {"l_gripper", "ball"}, OT_eq, {1e1});

//  komo.addContact_slide(s.phase0, s.phase1, s.frames(0), s.frames(1));
//  if(s.phase1>=s.phase0+.8){
//    rai::Frame* obj = komo.world.getFrame(s.frames(1));
//    if(!(obj->shape && obj->shape->type()==ST_sphere) && obj->children.N){
//      obj = obj->children.last();
//    }
//    if(obj->shape && obj->shape->type()==ST_sphere){
//      double rad = obj->shape->radius();
//      arr times = {s.phase0+.2,s.phase1-.2};
//      if(komo.k_order==1) times = {s.phase0, s.phase1};
//      komo.addObjective(times, make_shared<F_PushRadiusPrior>(rad), s.frames, OT_sos, {1e1}, NoArr, 1, +1, 0);
//    }
//  }
//  if(komo.k_order>1){
//    komo.addObjective({s.phase0, s.phase1}, FS_position, {s.frames(1)}, OT_sos, {3e0}, {}, 2); //smooth obj motion
//    komo.addObjective({s.phase1}, FS_pose, {s.frames(0)}, OT_eq, {1e0}, {}, 1);
//    komo.addObjective({s.phase1}, FS_pose, {s.frames(1)}, OT_eq, {1e0}, {}, 1);
//  }

#if 0 //for development only
  komo.optimize();
  cout <<komo.getReport(true);
  komo.pathConfig.gl()->ensure_gl().resize(700,500);
  komo.view(true, "solution");
  while(komo.view_play(true, .2, "z.vid/"));
  return;
#endif

  bool useOptitrack=rai::getParameter<bool>("bot/useOptitrack", false);

  SecMPC_Experiments ex(C, komo, .03, 1e0, .7, false);
  ex.step();
  ex.mpc->timingMPC.backtrackingTable=uintA{0, 0, 0, 0, 0, 0, 0};

  while(ex.step()){
    if(useOptitrack){
      C[obj]->setPosition(C["marc_red"]->getPosition());
      C[target]->setPosition(C["marc_green"]->getPosition());
      C["obst"]->setPose(C["HandStick"]->getPose());
    }
  }
}
