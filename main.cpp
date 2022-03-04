
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
#include "ex_hunting.cpp"

//===========================================================================

int main(int argc, char *argv[]){
  rai::initCmdLine(argc, argv);

//  rai::setParameter<rai::String>("log", "z.22-02-21--12-42-43.secMPC.log");

  if(rai::checkParameter<bool>("log")){
    playLog("z.SecMPC.log"); //rai::getParameter<rai::String>("log"));
    return 0;
  }

//  rnd.clockSeed();
  rnd.seed(1);

//  testBallFollowing();
//  testBallReaching();
//  testPnp();
  testPushing();
//  testPushing2();
//  testDroneRace();

//  ex_hunting();
//  ex_huntingStats(false, 0.);

  //-- baselines
//  ex_1DApproach();
//  phase_TimeOpt(true);
//  phase_LQR(true, true);
//  needleThreading2D();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
