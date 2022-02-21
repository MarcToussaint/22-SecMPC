
//standard way to setup an experiment
#include "experiment.h"

//concrete experiments
#include "ex_ballFollowing.cpp"
#include "ex_ballReaching.cpp"
#include "ex_pnp.cpp"
#include "ex_pushing.cpp"
#include "ex_pushing2.cpp"
#include "ex_droneRace.cpp"
#include "ex_baselines.cpp"

//===========================================================================

int main(int argc, char *argv[]){
  rai::initCmdLine(argc, argv);

//  rai::setParameter<rai::String>("log", "z.22-02-21--12-42-43.secMPC.log");

  if(rai::checkParameter<rai::String>("log")){
    playLog(rai::getParameter<rai::String>("log"));
    return 0;
  }

//  rnd.clockSeed();
  rnd.seed(1);

  testBallFollowing();
//  testBallReaching();
//  testPnp();
//  testPushing();
//  testPushing2();
//  testDroneRace();

  //-- baselines
//  ex_1DApproach();
//  phase_TimeOpt(true);
//  phase_LQR(true, true);

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
