#pragma once

#include <BotOp/bot.h>
#include <BotOp/SecMPC.h>
#include <Core/thread.h>

//===========================================================================

struct SecMPC_Experiments{
  rai::Configuration& C;
  unique_ptr<SecMPC> mpc;
  unique_ptr<BotOp> bot;
  Metronome tic;
  uint stepCount = 0;

  KOMO *__komo=0;
  double timeCost=1e0, ctrlCost=1e0;

  SecMPC_Experiments(rai::Configuration& _C, ObjectiveL& phi, double cycleTime=.1)
    : C(_C),
      tic(cycleTime)
      {
  }

  SecMPC_Experiments(rai::Configuration& _C, KOMO& komo, double cycleTime=.1, double timeCost=1e1, double ctrlCost=1e-2)
    : C(_C),
      tic(cycleTime),
      __komo(&komo),
      timeCost(timeCost), ctrlCost(ctrlCost){
  }

  bool step(ObjectiveL& phi);
};

//===========================================================================

//helper
void randomWalkPosition(rai::Frame* f, arr& centerPos, arr& velocity, double rate=.001);
