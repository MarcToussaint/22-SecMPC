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

  KOMO& komo;
  double timeCost=1e0, ctrlCost=1e0;

  SecMPC_Experiments(rai::Configuration& _C, KOMO& _komo, double cycleTime=.1, double timeCost=1e0, double ctrlCost=1e0)
    : C(_C),
      tic(cycleTime),
      komo(_komo),
      timeCost(timeCost), ctrlCost(ctrlCost){
  }

  bool step(ObjectiveL& phi);
};

//===========================================================================

//helper
void randomWalkPosition(rai::Frame* f, arr& centerPos, arr& velocity, double rate=.001);
