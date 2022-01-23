#include "experiment.h"

bool SecMPC_Experiments::step(ObjectiveL& phi){
  stepCount++;

  //-- start a robot thread
  if(!bot){
    bot = make_unique<BotOp>(C, rai::checkParameter<bool>("real"));
    bot->home(C);
    bot->setControllerWriteData(1);
    if(bot->optitrack) bot->optitrack->pull(C);
    rai::wait(.2);
  }

  if(!mpc){
    //needs to be done AFTER bot initialization (optitrack..)
    mpc = make_unique<SecMPC>(komo, C.getJointState(), timeCost, ctrlCost);
  }

  //-- iterate
  tic.waitForTic();

  //-- get optitrack
  if(bot->optitrack) bot->optitrack->pull(C);

  //-- get current state (time,q,qDot)
  arr q,qDot, q_ref, qDot_ref;
  double ctrlTime = 0.;
  bot->getState(q, qDot, ctrlTime);
  bot->getReference(q_ref, qDot_ref, NoArr, q, qDot, ctrlTime);

  //-- iterate MPC
  mpc->cycle(C, phi, q_ref, qDot_ref, q, qDot, ctrlTime);
  mpc->report(C, phi);

  //-- send spline update
  auto sp = mpc->getSpline(bot->get_t());
  if(sp.pts.N) bot->move(sp.pts, sp.vels, sp.times, true);

  //-- update C
  bot->step(C, .0);
  if(bot->keypressed=='q' || bot->keypressed==27) return false;

  return true;
}

//===========================================================================

void randomWalkPosition(rai::Frame* f, arr& centerPos, arr& velocity, double rate){
  arr pos = f->getPosition();
  if(!centerPos.N) centerPos = pos;
  if(!velocity.N) velocity = zeros(pos.N);
  rndGauss(velocity, rate, true);
  pos += velocity;
  pos = centerPos + .9 * (pos - centerPos);
  f->setPosition(pos);
}
