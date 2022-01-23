
//standard way to setup an experiment
#include "experiment.h"

//concrete experiments
#include "ex_ballFollowing.cpp"
#include "ex_ballReaching.cpp"
#include "ex_pnp.cpp"
#include "ex_pushing.cpp"
#include "ex_droneRace.cpp"

//===========================================================================

int main(int argc, char *argv[]){
  rai::initCmdLine(argc, argv);

  //  rnd.clockSeed();
  rnd.seed(1);

  //testBallFollowing();
  //testBallReaching();
  testPnp();
  //testPushing();
  //testDroneRace();

  LOG(0) <<" === bye bye ===\n used parameters:\n" <<rai::getParameters()() <<'\n';

  return 0;
}
