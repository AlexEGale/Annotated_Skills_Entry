// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// LineTrackerBottom    line          A               
// RangeFinderA         sonar         C, D            
// RangeFinderC         sonar         E, F            
// LineTrackerTop       line          B               
// ---- END VEXCODE CONFIGURED DEVICES ----


/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ALEX                                                      */
/*    Created:      Tue Apr 21 2020                                           */
/*    Description:  Skills Project V 3                                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "changeUpRobot.h"

using namespace vex;



#include <vex_triport.h>

#include <vex_imu.h>
// A global instance of competition
//competition Competition;

vex::competition Competition;
bot robot = bot();

void pre_auton(void)
{

  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  robot.Calibrate_Inertial_Sensor();


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}


//____________________________________________________________________________________________________________________________________________________________||
//                                                                                Auton                                                                       ||
//____________________________________________________________________________________________________________________________________________________________||


void autonomous(void)
{
  robot.test();
 // robot.eko("yay i did something.");
  //robot.wahyt();
} // auton


//____________________________________________________________________________________________________________________________________________________________||
//                                                                                Driver                                                                      ||
//____________________________________________________________________________________________________________________________________________________________||

void usercontrol(void)
{
  robot.usercontrolled();
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  
}
