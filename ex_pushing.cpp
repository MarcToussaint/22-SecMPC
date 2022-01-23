#include "experiment.h"

#include <Kin/F_forces.h>

void testPushing() {
  rai::Configuration C;
  C.addFile("pushScenario.g");


  //-- define constraints
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(3., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addObjective({1.}, make_shared<F_PushRadiusPrior>(.13), {"stickTip", "puck", "target"}, OT_eq, {1e1}, {0., 0., .1});
  komo.addObjective({2.}, make_shared<F_PushRadiusPrior>(.10), {"stickTip", "puck", "target"}, OT_eq, {1e1});
  komo.addObjective({3.}, make_shared<F_PushRadiusPrior>(.02), {"stickTip", "puck", "target"}, OT_eq, {1e1});
  komo.addObjective({1., 3.}, make_shared<F_PushAligned>(), {"stickTip", "puck", "target"}, OT_eq, {{1,3},{0,0,1e1}});
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

  bool useOptitrack=rai::getParameter<bool>("bot/useOptitrack", false);

  SecMPC_Experiments ex(C, komo);
  ex.step(komo.objectives);
  ex.mpc->timingMPC.backtrackingTable=uintA{0, 0, 0, 0, 0};

  while(ex.step(komo.objectives)){
    if(useOptitrack){
      C["puck"]->setPosition(C["b1"]->getPosition());
    }
  }
}
