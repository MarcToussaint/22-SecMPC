#include "experiment.h"

void testDroneRace(){
  rai::Configuration C;
  C.addFile("droneRace.g");

  //-- define constraints
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(7., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addQuaternionNorms();
  komo.addObjective({1.}, FS_positionDiff, {"drone", "target0"}, OT_eq, {1e1});
  komo.addObjective({2.}, FS_positionDiff, {"drone", "target1"}, OT_eq, {1e1});
  komo.addObjective({3.}, FS_positionDiff, {"drone", "target2"}, OT_eq, {1e1});
  komo.addObjective({4.}, FS_positionDiff, {"drone", "target3"}, OT_eq, {1e1});
  komo.addObjective({5.}, FS_positionDiff, {"drone", "target0"}, OT_eq, {1e1});
  komo.addObjective({6.}, FS_positionDiff, {"drone", "target1"}, OT_eq, {1e1});
  komo.addObjective({7.}, FS_position, {"drone"}, OT_eq, {1e1}, {0,-.5, 1.});


  arrA targetCen(4), targetVel(4);

#if 1
  //-- reactive control
  SecMPC_Experiments ex(C, komo, .1, 1e0, 1e0);
  ex.step(komo.objectives);
  ex.mpc->tauCutoff = .1;

  //void CubicSplineCtrlReference::getReference(arr& q_ref, arr& qDot_ref, arr& qDDot_ref, const arr& q_real, const arr& qDot_real, double ctrlTime){

  while(ex.step(komo.objectives)){
    if(ex.mpc->timingMPC.phase==5){ //hard code endless loop by phase backtracking
      ex.mpc->timingMPC.update_setPhase(1);
    }
    for(uint g=0;g<2;g++){
      rai::Frame *target = C[STRING("target"<<g)];
      randomWalkPosition(target, targetCen(g), targetVel(g), .003);
    }
  }

#else
  //-- manually just optimize once and dump spline
  //optimize keyframes
  komo.optimize();
  komo.getReport(true);
  komo.view(false, "optimized motion");
  arr keyframes = komo.getPath_qOrg();

  //optimize timing
  TimingMPC F(keyframes, 1e0, 10); //last number (ctrlCost) in range [1,10] from fast-slow
  arr x0 = C["drone"]->getPosition();
  arr v0 = zeros(3);
  F.solve(x0, v0);

  //get spline
  rai::CubicSpline S;
  F.getCubicSpline(S, x0, v0);

  //analyze only to plot the max vel/acc
  arr path = S.eval(range(0., S.times.last(), 100));
  double tau = S.times.last()/100.;
  arr ttau = consts<double>(tau, 101);
  double maxVel=1., maxAcc=1., maxJer=30.;
  arr time(path.d0);
  time.setZero();
  for(uint t=1;t<time.N;t++) time(t) = time(t-1) + ttau(t);

  arr v = max(getVel(path,ttau),1) / maxVel;
  arr a = max(getAcc(path,ttau),1) / maxAcc;
  arr j = max(getJerk(path,ttau),1) / maxJer;
  arr vi = min(getVel(path,ttau),1) / maxVel;
  arr ai = min(getAcc(path,ttau),1) / maxAcc;
  arr ji = min(getJerk(path,ttau),1) / maxJer;
  catCol(LIST(~~time, ~~v, ~~a, ~~j, ~~vi, ~~ai, ~~ji)).reshape(-1,7).writeRaw( FILE("z.dat") );
  gnuplot("plot [:][-1.1:1.1] 'z.dat' us 1:2 t 'vmax', '' us 1:3 t 'amax', '' us 1:5 t 'vmin', '' us 1:6 t 'amin'"); //, '' us 1:4 t 'j', , '' us 1:7 t 'jmin'

  //display
  rai::Mesh M;
  M.V = S.eval(range(0., S.times.last(), 100));
  M.makeLineStrip();
  C.gl()->add(M);
  C.watch(true);


  //just sample & dump the spline
  for(double t=0;t<S.times.last();t += .01){
    //time 3-positions 3-velocities
    cout <<t <<S.eval(t).modRaw() <<' ' <<S.eval(t,1).modRaw() <<endl;
  }
#endif

}

