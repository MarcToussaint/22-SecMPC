#include "experiment.h"

//===========================================================================

void ex_1DApproach(){
  rai::Configuration C;
  C.addFile("model1d.g");
  C.watch();

  C.setJointState({1.});
  arr qHome = C.getJointState();

  //-- define constraints
  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(1., 1, 5., 1);
  komo.add_qControlObjective({}, 1, 1e-1);
  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e1});

  SecMPC_Experiments ex(C, komo, .1, 1e0, 0.5, false); //LAST ARGUMENT: NO AUTO-TANGENTS!
  while(ex.step()){}

//  SecMPC mpc(C);

//  ofstream fil("z.dat");
//  for(uint k=0;k<100;k++){
//    mpc.reinit(C);
//    mpc.solve();
//    cout <<"PATH: " <<mpc.xT <<" \t timing: " <<mpc.tau <<endl;
//    fil <<mpc.xT.modRaw() <<' ' <<mpc.tau.modRaw() <<endl;
//  }
//  gnuplot("plot 'z.dat' us 0:1, '' us 0:5");
//  rai::wait();
}

//===========================================================================

void phase_TimeOpt(bool zeroVel){
//  rai::Configuration C;
//  C.addFile("model1d.g");
  arr q0 = {0.};

//  SecMPC mpc(komo, 0, -1, 1e0, 0.5, false);
  TimingMPC mpc(~q0 , 1e0, 0.5);
  rai::CubicSpline sp;

  ofstream fil("z.phase.mpc");
  for(double v=-5.;v<=5.;v+=.5){
    if(zeroVel) v=1.;
    for(double x=-5.;x<=5.;x+=(zeroVel?.05:.5)){

      mpc.solve({x}, {v}, 1);

      mpc.getCubicSpline(sp, {x}, {v});

#if 0 //plotting the solution
      arr path = sp.eval(range(0., sp.times.last(), 100));
      FILE("z.path") <<path.modRaw();
      gnuplot("plot 'z.path' us 0:1");
      rai::wait();
#endif

      double timeToGo = sp.times.last();

      double tau=.1;
      arr q, qDot, qDDot, qTau, qDotTau;
      sp.eval(q, qDot, qDDot, 0.);
      sp.eval(qTau, qDotTau, NoArr, tau);
      fil <<x <<' ' <<v <<' ' <<q.modRaw() <<' ' <<qDot.modRaw() <<' ' <<qDDot.modRaw() <<' ' <<qTau.modRaw() <<' ' <<qDotTau.modRaw() <<' ' <<timeToGo <<endl;
    }
    if(zeroVel) break;
  }

  gnuplot("load 'plt.phase'", true);
}


void phase_LQR(double zeroVel, bool clipped){
  double kp=2., kd=2.*::sqrt(kp); //critically damped
  double tau=.1;
  double delta=.01;  //e^-(t*kd/2)*x = d   -> - 2log(d/x)/kd
  ofstream fil(STRING("z.phase.pd"<<(clipped?"c":"")));
  for(double v=-5.;v<=5.;v+=.5){
    if(zeroVel) v=1.;
    for(double x=-5.;x<=5.;x+=(zeroVel?.05:.5)){
      double x_ = x;
      if(clipped) rai::clip(x_, -2., 2.);
      double a = -kp*x_ -kd*v;

      double d = fabs(x)/delta;
      double timeToGo = 2.*log((d>1.?d:1.))/kd;

      double v1 = v + tau*a;
      double x1 = x + tau*v + .5*a*tau*tau;
      //fil <<x <<' ' <<v <<' ' <<x1 <<" 0 0 " <<tau <<endl;
      fil <<x <<' ' <<v <<' ' <<x <<' ' <<v <<' ' <<a <<' ' <<x1 <<' ' <<v1 <<' ' <<timeToGo <<endl;
    }
    if(zeroVel) break;
  }
  gnuplot("load 'plt.acc'", true);
}

//===========================================================================

void needleThreading2D(){
  arr waypoints = {{2,2}, {0,0, 0, 1}};

//  SecMPC mpc(komo, 0, -1, 1e0, 0.5, false);
  TimingMPC mpc(waypoints , 1., 1.);
  rai::CubicSpline sp;

  ofstream fil("z.path");
  for(double pos=0.;pos<=.8;pos+=.05){
    arr x = {.1, -1.+pos};
    arr v = {0., 2.};
    mpc.tau = 10.*ones(waypoints.d0);
    mpc.solve(x, v, 1);

    mpc.getCubicSpline(sp, {x}, {v});

#if 1 //plotting the solution
    arr path = sp.eval(range(0., sp.times.last(), 100));
    fil <<path.modRaw() <<"\n" <<endl;
    gnuplot("plot [-.5:.5] 'z.path' us 1:2");
    rai::wait();
#endif

    double timeToGo = sp.times.last();

//      double tau=.1;
//      arr q, qDot, qDDot, qTau, qDotTau;
//      sp.eval(q, qDot, qDDot, 0.);
//      sp.eval(qTau, qDotTau, NoArr, tau);
//      fil <<x <<' ' <<v <<' ' <<q.modRaw() <<' ' <<qDot.modRaw() <<' ' <<qDDot.modRaw() <<' ' <<qTau.modRaw() <<' ' <<qDotTau.modRaw() <<' ' <<timeToGo <<endl;
//    }
//    if(zeroVel) break;
  }

//  gnuplot("load 'plt.phase'", true);
}


//===========================================================================
#if 0

void testPath(){
  rai::Configuration C;
  C.addFile("model1d.g");
  C.watch();

  C.setJointState({1.});
  C.addTauJoint();
  auto timeF = C.frames.first();
  timeF->ats->newNode<bool>("constant", {}, true);

  KOMO komo;
  komo.setModel(C, false);
  komo.setTiming(1., 100, 2.45, 2);
//  komo.add_qControlObjective({}, 2, 1e0);
  //control costs equivalent - just for checking:
  auto o = komo.addObjective({}, FS_qItself, {"bot"}, OT_sos, {1.}, NoArr, 2);
  o->feat->timeIntegral=1;

  komo.addObjective({}, make_shared<F_qTime>(), {timeF->name}, OT_f, {1e-0}, {}); //optimize for min time

  //target position:
  komo.addObjective({1}, FS_qItself, {"bot"}, OT_eq, {1e2}, NoArr);
  komo.addObjective({1}, FS_qItself, {"bot"}, OT_eq, {1e2}, NoArr, 1);

  komo.reportProblem();

  OptOptions opt;
  opt.stopTolerance = 1e-4;
  opt.stopGTolerance = 1e-4;
//    opt.stopIters = 20;
  //  opt.nonStrictSteptestPaths=-1;
//    opt.maxStep = .1; //*tau; //maxVel*tau;
//    opt.damping = 1e-1;
  //  komo.verbose=4;
  komo.opt.verbose=4;
//    komo.animateOptimization=animate;
  komo.optimize(0., opt);
  //komo.checkGradients();
  //  komo.plotTrajectory();
  cout <<"PATH: " <<komo.x <<" \t ";
  cout <<"timing: " <<komo.getPath_tau() <<endl;
  FILE("z.dat") <<(~~komo.x).modRaw() <<endl;

  arr q = komo.getPath_qOrg();
  arr taus = komo.getPath_tau();
  double tau = sum(taus)/taus.N;
  arr t = integral(taus);
  arr v = getVelocities_centralDifference(q, tau);
  arr a = getAccelerations_centralDifference(q, tau);

  FILE("z.dat") <<catCol(t, q, v, a).modRaw();
  gnuplot("plot 'z.dat' us 1:2, '' us 1:3, '' us 1:4");

  komo.view(false, "optimized motion");
  komo.view(true, "optimized motion");
  while(komo.view_play(true));

  rai::wait();
}

//===========================================================================
#endif
