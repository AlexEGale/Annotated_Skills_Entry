#include "changeUpRobot.h"

//#include "vex.h"

// these are functions for autonomous we'd be using in-match, for the 15-second auton section. 

void bot::left1() // because we're all about that base
{
    float sped = 50;
    bot::driveSmooth(2, sped, distUnit::tiles);
    bot::driveSmooth(-1, sped, distUnit::tiles);
    bot::turnPid(-135, sped, rotationUnit::degrees);
    bot::driveSmooth(0.6, sped, distUnit::tiles);
    
}
void bot::left2() // because we're all about that base
{
    float sped = 50;
    bot::driveSmooth(2, sped, distUnit::tiles); 
    bot::driveSmooth(-1, sped, distUnit::tiles);
    bot::turnPid(-135, sped, rotationUnit::degrees);
    bot::driveSmooth(0.6, sped, distUnit::tiles);
    //neeext stage
    bot::turnPid(-135, sped, rotationUnit::degrees);
    bot::driveSmooth(1.2, sped, distUnit::tiles);
    bot::turnPid(20, sped, rotationUnit::degrees);

}

void bot::right1() // because we're all about that base
{
    float sped = 50;
    bot::driveSmooth(2, sped, distUnit::tiles);
    bot::driveSmooth(-1, sped, distUnit::tiles);
    bot::turnPid(135, sped, rotationUnit::degrees);
    bot::driveSmooth(0.6, sped, distUnit::tiles);
    
}
void bot::right2() // because we're all about that base
{
    float sped = 50;
    bot::driveSmooth(2, sped, distUnit::tiles);
    bot::driveSmooth(-1, sped, distUnit::tiles);
    bot::turnPid(135, sped, rotationUnit::degrees);
    bot::driveSmooth(0.6, sped, distUnit::tiles);
    //neeext stage
    bot::turnPid(135, sped, rotationUnit::degrees);
    bot::driveSmooth(1.2, sped, distUnit::tiles);
    bot::turnPid(-20, sped, rotationUnit::degrees);

}


void bot::test()
{
  bot::skillz();
}

void bot::usercontrolled()
{
  bot::Usercontrol();

}

void bot::wahyt(){

  bot::wait(10);
}

void bot::eko(const char *message){

  bot::echo(message);
}

void bot::Calibrate_Inertial_Sensor() 
{
  bot::inertialcalibrate();
}





