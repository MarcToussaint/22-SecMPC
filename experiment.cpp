#include "experiment.h"

#    include <GL/glew.h>
#    include <GL/glut.h>

#include <OptiTrack/optitrack.h>
#include <Gui/opengl.h>
#include <Kin/viewer.h>


bool SecMPC_Experiments::step(){
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
    mpc = make_unique<SecMPC>(komo, subSeqStart, subSeqStop, timeCost, ctrlCost, setNextWaypointTangent);
    mpc->tauCutoff = 2.*tic.ticInterval;
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
  mpc->cycle(C, q_ref, qDot_ref, q, qDot, ctrlTime);
  mpc->report(C);
  if(mpc->phaseSwitch) bot->sound(7 * mpc->timingMPC.phase);

  if(fil.is_open()){
    fil <<stepCount <<' ' <<ctrlTime <<' ' <<q.modRaw() <<' ' <<mpc->timingMPC.phase <<endl;
  }

  //-- send spline update
  bot->getState(q, qDot, ctrlTime);
//  auto sp = mpc->getSpline(ctrlTime);
  auto sp = mpc->getShortPath(ctrlTime);
  if(sp.pts.d0){
//    if(sp.times.first()<0.) bot->sound(2*(stepCount%12));
    if(sp.vels.N) bot->move(sp.pts, sp.vels, sp.times, true, ctrlTime);
    else bot->move(sp.pts, sp.times, true, ctrlTime);
  }

  //-- update C
  bot->step(C, .0);
  if(bot->keypressed=='q' || bot->keypressed==27) return false;

  //if(mpc->timingMPC.done()) return false;

  return true;
}

void SecMPC_Experiments::selectSubSeq(int _subSeqStart, int _subSeqStop){
  mpc.reset(); //kill the current mpc - created again in next cycle
  subSeqStart = _subSeqStart;
  subSeqStop = _subSeqStop;
}

//===========================================================================

void randomWalkPosition(rai::Frame* f, arr& centerPos, arr& velocity, double rate){
  arr pos = f->getPosition();
  if(!centerPos.N) centerPos = pos;
  if(!velocity.N) velocity = zeros(pos.N);
  rndGauss(velocity, rate, true);
  velocity *= .99;
  pos += velocity;
  pos = centerPos + .9 * (pos - centerPos);
  f->setPosition(pos);
}

//===========================================================================


void playLog(const rai::String& logfile){
  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandasTable-calibrated.g"));

  arr dat;
  dat <<FILE(logfile);
  cout <<dat <<endl;

  rai::String text;
  C.gl()->ensure_gl().add([&text](OpenGL& gl){
    glColor(.8,.8,.8,.5);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glOrtho(0., (double)gl.width, (double)gl.height, .0, -1., 1.);
    glDrawText(text, 20, gl.height-20, 0, true);
  });

  for(uint t=0;t<dat.d0;t++){
    C.setJointState(dat(t, {2,2+13}));
    text.clear() <<"phase: " <<dat(t,-1) <<" ctrlTime:" <<dat(t,1);
    C.watch(true);
  }

};
