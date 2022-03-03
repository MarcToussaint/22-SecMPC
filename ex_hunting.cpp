#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <Optim/MP_Solver.h>
#include <Control/timingMPC.h>
#include <KOMO/pathTools.h>

#include <thread>

//===========================================================================

double ex_hunting(bool fullSequence=true, double noise=.05, int verbose=1){
  //-- create random waypoints
  uint K=5, d=3;
  arr waypoints(K,d);
  rndUniform(waypoints, -.5, .5);
  for(uint k=0;k<K;k++) waypoints(k,2) += 1.;

  TimingMPC F(waypoints, 1e0);

  rai::Configuration C;
  for(uint k=0;k<K;k++){
    rai::Frame *f = C.addFrame(STRING("flag_"<<k));
    f->setShape(rai::ST_sphere, {.02});
    f->setColor({.2, .2, .2});
    f->setPosition(F.waypoints[k]);
  }

  rai::Frame *bot = C.addFrame(STRING("ego"));
  bot->setShape(rai::ST_sphere, {.02}).setColor({.8, .6, .6});

  //-----------------------

  //start iteration
  F.solve(zeros(3), zeros(3), verbose);

  //read out spline
  rai::CubicSpline S;
  F.getCubicSpline(S, zeros(3), zeros(3));

  //illustration in gl
  rai::Mesh M;
  M.V = S.eval(range(0., S.times.last(), 100));
  M.makeLineStrip();
  C.gl()->add(M);
  if(verbose>0) C.watch(true);

  double del=.001, spineTime=0., time=0.;
  uint PHASE = 0;

  for(uint step=0;;step++, spineTime+=del, time+=del){
    arr x0 = S.eval(spineTime);
    arr v0 = S.eval(spineTime,1);
    bot->setPosition(x0);

    //phase progression
    if(maxDiff(x0, waypoints[PHASE]) < .02){
      C.frames(PHASE)->setColor({.2,.8,.2});
      PHASE++;
      if(PHASE==K) break;
      if(fullSequence){
          F.phase ++;
      }
    }

    if(!(step%10)) C.watch();
    if(verbose>0) rai::wait(del);

    //-- perturb waypoints
    if(rnd.uni()<2.*del){
      uint m=rnd(waypoints.d0);
      m=PHASE;
      waypoints[m] += noise*randn(waypoints.d1);
      C.frames(m)->setPosition(waypoints[m]);
      if(fullSequence){
        F.update_waypoints(waypoints, false);
      }else{
        F.update_waypoints(waypoints({PHASE,PHASE}), false);
      }
    }

    //-- update:
    if(spineTime>.1){
      F.solve(x0, v0, verbose);
      F.getCubicSpline(S, x0, v0);
      M.V = S.eval(range(0., S.times.last(), 100));
      spineTime = .0;
    }

  }
  if(verbose>0) C.watch(true);

  return time;
}

//===========================================================================

void ex_huntingStats(bool fullSequence=true, double noise=.05){
    uint K=20;
    double Ttotal=0., Tsqr=0.;;
    for(uint k=0;k<K;k++){
        double T = ex_hunting(fullSequence, noise, 0);
        Ttotal += T;
        Tsqr += T*T;
        cout <<"T: " <<T <<endl;
    }
    double mean = Ttotal/double(K);
    double sdv = sqrt(Tsqr/double(K) - mean*mean);
    double md = sdv/sqrt(double(K));
    cout <<"mean:" <<mean <<" std:" <<sdv <<" err:" <<md <<endl;
}
