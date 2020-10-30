#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
line LineTrackerBottom = line(Brain.ThreeWirePort.A);
sonar RangeFinderA = sonar(Brain.ThreeWirePort.C);
sonar RangeFinderC = sonar(Brain.ThreeWirePort.E);
line LineTrackerTop = line(Brain.ThreeWirePort.B);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}