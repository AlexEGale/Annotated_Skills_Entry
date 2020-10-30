#include "Robot.h"

#include "vex.h"

//____________________________________________________Devices_________________________________________________
//vex::brain Brain;
//vex::competition Competition;


vex::controller Controller = vex::controller();
vex::controller Controller2 = vex::controller(vex::controllerType::partner);
vex::motor FRDrive = vex::motor(vex::PORT1);
vex::motor FLDrive = vex::motor(vex::PORT2);
vex::motor BLDrive = vex::motor(vex::PORT3);
vex::motor BRDrive = vex::motor(vex::PORT4);
vex::motor Roller1 = vex::motor(vex::PORT5);
vex::motor Roller2 = vex::motor(vex::PORT18);
// vex::motor Lift1 = vex::motor(vex::PORT20);
vex::motor Intake1 = vex::motor(vex::PORT13);
vex::motor Intake2 = vex::motor(vex::PORT19);
// vex::motor Tilter = vex::motor(vex::PORT8);
vex::timer Timer = vex::timer();
vex::timer Timer2 = vex::timer();
vex::timer Timer3 = vex::timer();
vex::timer Timer4 = vex::timer();
vex::inertial inertialSensor = vex::inertial(vex::PORT8);
vex::vision visionSensorlow = vex::vision(vex::PORT11);
vex::vision visionSensorhigh = vex::vision(vex::PORT12);
vex::vision Vision12 = vex::vision(vex::PORT6);


/*  Hello and Welcome to 371T's code! :D
.   In order to successfully peruse the many lines of code we have, you can travel to a specific section by pressing "Ctrl" and "F" at the same time and typing the tag
.   we put in parentheses next to the specific item in the contents. 
.   Example: Kung Fu (tag99) and then you'd Ctrl+F and type tag99 into the search bar, which will take you to the specific section. 
.   At any time, you can come back to the top just by typing tag and then any number from 1-7 or 99. It'll take you right to the top. :D
.   Enjoy our code!! :D

CONTENTS:

  Conversion code (tag1)
  Drive smooth -- also known as MoveIncrease (tag2)
  TurnPid (tag3)
  Intake functions (tag4)
  Other drive functions (tag5)
  Moveincrease functions (tag6)
  Skills function (tag7)
  MoveMinus function -- the rangefinder function(tag8)


*/


//_______________________________________________________________________________________________________________________ Essentials

  // In order for our code to be transversible between robots (say, we're doing a rebuild and need to begin using a completely different robot),
  // we created a function that converts revs (which is hardcoded and different per robot) into tiles, inches and feet (which are intuitive, easy to measure, and very versatile).
  
  // Every time we move, the distance is natively hardcoded in revs, but because we want ease of access and versatility, 
  // we made a translator that automatically converts revs to tiles, inches, or feet. 
float conversionCode(float distValue, distUnit units = distUnit::rev) // tag1
{


 
  float rev = distValue;
  switch (units)
  {
  case distUnit::rev:
    rev = distValue;
    break;
  case distUnit::tiles:
    rev = ((distValue * 24) / 12.57);
    break;
  case distUnit::feet:
    rev = ((distValue * 24) / 12.57) / 2;
    break;
  case distUnit::inches:
    rev = ((distValue * 24) / 12.57) / 12 / 2;
    break;
  default:
    rev = distValue;
    break;
  }

  return rev;
}

// __________________________________________________________________________________________________________________ USEful FUNCTiONS

void Robot::driveSmooth(float distValue, float maxSpeed, distUnit units) // tag2 _____________________________ Drive Smooth (AKA moveIncrease)
{
  // Drivesmooth is about rounding out the corners of the square wave that zero-to-200rpm gives. In order to stop the sudden jolts that sudden acceleration creates,
  // we use trigonometry to calculate a sine wave that allows us to slowly speed up and slow down. 

  // 

  FLDrive.resetRotation();
  FRDrive.resetRotation();
  BLDrive.resetRotation();
  BRDrive.resetRotation();

  vex::task::sleep(5);

  float speed;
  float currentRev = 0;
 
  float maxRev = conversionCode(distValue, units);
  float sign = (maxRev > 0 ? -1 : 1);
  float rightMult;
  float leftMult;
  float globalCurrentRev;
  rightMult = 1;
  leftMult = 1;

  float getToTop = 1.5;
  float getToZero = 3;
  float speedOffset = 15;

  Timer.clear();
  float timerNow = Timer.time(vex::timeUnits::sec);
  bool isTicked = false;
  while (fabs(currentRev) < fabs(maxRev))
  {
    
    currentRev = ((fabs(BLDrive.rotation(vex::rotationUnits::rev)) + fabs(FRDrive.rotation(vex::rotationUnits::rev)) + fabs(FLDrive.rotation(vex::rotationUnits::rev)) + fabs(BRDrive.rotation(vex::rotationUnits::rev))) / 4);
    globalCurrentRev = currentRev;
    speed = maxSpeed;

    if (currentRev < getToTop)
    {
      float timeSpeed = -(100 / 2) * cos((6.28318530718 / 3) * (Timer.time(vex::timeUnits::sec))) + (100 / 2);
      if (speed > timeSpeed)
      {
        speed = timeSpeed;
      }
    }
    else if (fabs(maxRev) - currentRev < getToZero)
    {
      float projectedCurrentRev = currentRev + 0.05;

      float maxSlopeSpeed = (0.5 * cos((6.28318530718 / (getToZero * 2)) * ((fabs(maxRev) - fabs(projectedCurrentRev)) + (getToZero))) + 0.5) * 100 + speedOffset;

      if (speed > maxSlopeSpeed)
      {
        speed = maxSlopeSpeed;
      }
    }

    float minSpeed = 3;

    if (speed < minSpeed)
    {
      speed = minSpeed;
    }

    //speed = 100;

    BLDrive.spin(directionType::fwd, speed * sign * leftMult, velocityUnits::pct);
    FLDrive.spin(directionType::fwd, speed * sign * leftMult, velocityUnits::pct);
    BRDrive.spin(directionType::rev, speed * sign * rightMult, velocityUnits::pct);
    FRDrive.spin(directionType::rev, speed * sign * rightMult, velocityUnits::pct);

    echonum(speed*sign*leftMult);
    //return;

    while (FLDrive.isSpinning() || FRDrive.isSpinning() || BLDrive.isSpinning() || BRDrive.isSpinning())
    {

      if (!isTicked && (!FLDrive.isSpinning() || !FRDrive.isSpinning() || !BLDrive.isSpinning() || !BRDrive.isSpinning()))
      {

        isTicked = true;

        timerNow = Timer.time(vex::timeUnits::sec) + 0.1;
      }
      if (Timer.time(vex::timeUnits::sec) > timerNow)
      {
        break; // get the heck out of the function NOWWW
      }
      vex::task::sleep(5);
    }
  }
  BLDrive.stop();
  BRDrive.stop();
  FLDrive.stop();
  FRDrive.stop();
}

// Printing to screen/controller function section

  // in the case that we need to debug something in the code, we can add printing functions into the code so that we know where something crashed or malfunctioned.
  void printController(const char *message)
  {
    Controller.Screen.clearLine(2);
    Controller.Screen.setCursor(2, 0);
    Controller.Screen.print(message);
  }
  void printControllerNum(float num)
  {
    Controller.Screen.clearLine(2);
    Controller.Screen.setCursor(2, 0);
    Controller.Screen.print(num);
  }

  void Robot::echonum(float num) {
    printControllerNum(num);
  }

  void Robot::echo(const char *message) {
    printController(message);
  }

  void printControllerValue(float value)
  {
  Controller.Screen.clearLine(2);
  Controller.Screen.setCursor(2, 0);
  Controller.Screen.print(value);
  }

//

float em = 0.61;

//turning tag3
void turnPid(float rotationValue, float TurnSpeed, rotationUnit units = rotationUnit::rev, int count = 0, bool absoluteTurn = false, float timeout = 60) // for internal use only
{
  float startDeg = inertialSensor.heading(degrees); // reads the current inertial sensor heading (where we're facing)

  startDeg = startDeg > 180 ? startDeg - 360 : startDeg; // makes the inertial sensor value between 180 and -180, so we can say "turn to -90" rather than "turn to 270"
 
  if (absoluteTurn)
  {
    rotationValue = rotationValue - startDeg;
    rotationValue = rotationValue > 180 ? rotationValue - 360 : rotationValue;
    rotationValue = rotationValue < -180 ? rotationValue + 360 : rotationValue;
  }
  if (fabs(rotationValue) > 180)
  {
    /*
    printControllerValue(rotationValue);*/
  }

  float rev = rotationValue;
  switch (units)
  {
  case rotationUnit::rev:
    rev = rotationValue;
    break;
  case rotationUnit::degrees:
    rev = (rotationValue / 90) * em; 
    break;
  default:
    rev = rotationValue;
    break;
  } 
  
  float timerNow = Timer.time(vex::timeUnits::sec) + 60;
  bool isTicked = false;

  float adjTurn;

  FRDrive.startRotateFor(rev, vex::rotationUnits::rev, TurnSpeed, vex::velocityUnits::pct);
  FLDrive.startRotateFor(rev, vex::rotationUnits::rev, TurnSpeed, vex::velocityUnits::pct);
  BLDrive.startRotateFor(rev, vex::rotationUnits::rev, TurnSpeed, vex::velocityUnits::pct);
  BRDrive.startRotateFor(rev, vex::rotationUnits::rev, TurnSpeed, vex::velocityUnits::pct);
  while ((FLDrive.isSpinning() || FRDrive.isSpinning() || BLDrive.isSpinning() || BRDrive.isSpinning()) && Timer.time(vex::timeUnits::sec) < timeout)
  {

    if (!isTicked && (!FLDrive.isSpinning() || !FRDrive.isSpinning() || !BLDrive.isSpinning() || !BRDrive.isSpinning()))
    {

      isTicked = true;

      timerNow = Timer.time(vex::timeUnits::sec) + 0.1;
    }
    if (Timer.time(vex::timeUnits::sec) > timerNow)
    {
      break; // get the heck out of the function NOWWW
    }
    vex::task::sleep(20);
  }
  vex::task::sleep(150); //was 100 
  float currentDeg = inertialSensor.heading(degrees);

  //this fixes turns to fit into -180 to +180 for currentDeg

  currentDeg = currentDeg > 180 ? currentDeg - 360 : currentDeg;
  //printControllerValue(currentDeg);
  float turnValue = currentDeg - startDeg;
  float turnGoal = rotationValue; //if its degrees bro

  //
  float turnAmount = turnGoal - turnValue;
  turnAmount = turnAmount > 180 ? turnAmount - 360 : turnAmount;
  turnAmount = turnAmount < -180 ? turnAmount + 360 : turnAmount;


  // lets go to the testing port
  adjTurn = turnAmount;

  Brain.Screen.print(adjTurn);
  Brain.Screen.print(", ");

  if (fabs(adjTurn) > 5) //was 5
  {

    if (fabs(turnAmount) > 180)
    {
      /*printControllerValue(turnAmount);*/
    }

    if (count == 0)
    {
      Timer3.clear();
    }

    if (Timer3.time(vex::timeUnits::sec) > 1)
    {
      return;
    }

    if (count < 1) //count was 4
    {
      vex::task::sleep(200);
      turnPid(adjTurn, 15, rotationUnit::degrees, count + 1);
    }
  }
  //add 1 each time it recurses so you can get out if necessary
} 

void Robot::turnPid(float rotationValue, float TurnSpeed, rotationUnit units, int count, bool absoluteTurn, float timeout) // for external use (check changeUpRobot.cpp)
{
  // In order to turn accurately, we use an inertial sensor! An inertial sensor can tell you how far you've turned from a specific orientation, 
  // so we calibrate it at the beginning of autonomous and calculate our turns based on that initial measurement. In addition to enabling accurate turns, 
  // it allows us to make absolute turns, turns that are measured from the initial calibration orientation.

  float startDeg = inertialSensor.heading(degrees);

  startDeg = startDeg > 180 ? startDeg - 360 : startDeg;

  if (absoluteTurn)
  {
    rotationValue = rotationValue - startDeg;
    rotationValue = rotationValue > 180 ? rotationValue - 360 : rotationValue;
    rotationValue = rotationValue < -180 ? rotationValue + 360 : rotationValue;
  }

  float rev = rotationValue;
  switch (units)
  {
  case rotationUnit::rev:
    rev = rotationValue;
    break;
  case rotationUnit::degrees:
    rev = (rotationValue / 90) * em; //unkt
    break;
  default:
    rev = rotationValue;
    break;
  } 

  float timerNow = Timer.time(vex::timeUnits::sec) + 60;
  bool isTicked = false;

  float adjTurn;

  FRDrive.startRotateFor(-rev, vex::rotationUnits::rev, TurnSpeed, vex::velocityUnits::pct);
  FLDrive.startRotateFor(-rev, vex::rotationUnits::rev, TurnSpeed, vex::velocityUnits::pct);
  BLDrive.startRotateFor(-rev, vex::rotationUnits::rev, TurnSpeed, vex::velocityUnits::pct);
  BRDrive.startRotateFor(-rev, vex::rotationUnits::rev, TurnSpeed, vex::velocityUnits::pct);
  while ((FLDrive.isSpinning() || FRDrive.isSpinning() || BLDrive.isSpinning() || BRDrive.isSpinning()) && Timer.time(vex::timeUnits::sec) < timeout)
  {

    if (!isTicked && (!FLDrive.isSpinning() || !FRDrive.isSpinning() || !BLDrive.isSpinning() || !BRDrive.isSpinning()))
    {

      isTicked = true;

      timerNow = Timer.time(vex::timeUnits::sec) + 0.1;
    }
    if (Timer.time(vex::timeUnits::sec) > timerNow)
    {
      break;
    }
    vex::task::sleep(20);
  }
  vex::task::sleep(150); //was 100
  float currentDeg = inertialSensor.heading(degrees);

  //this fixes turns to fit into -180 to +180 for currentDeg

  currentDeg = currentDeg > 180 ? currentDeg - 360 : currentDeg;
  //printControllerValue(currentDeg);
  float turnValue = currentDeg - startDeg;
  float turnGoal = rotationValue;

  
  float turnAmount = turnGoal - turnValue;
  turnAmount = turnAmount > 180 ? turnAmount - 360 : turnAmount;
  turnAmount = turnAmount < -180 ? turnAmount + 360 : turnAmount;



  adjTurn = turnAmount;

  Brain.Screen.print(adjTurn);
  Brain.Screen.print(", ");

  if (fabs(adjTurn) > 5) //was 5
  {

    if (fabs(turnAmount) > 180)
    {
      /*printControllerValue(turnAmount);*/
    }

    if (count == 0)
    {
      Timer3.clear();
    }

    if (Timer3.time(vex::timeUnits::sec) > 1)
    {
      return;
    }

    if (count < 1) //count was 4
    {
      turnPid(adjTurn, 20, rotationUnit::degrees, count + 1); //magic number was 9
    }
  }
  //add 1 each time it recurses so you can get out if necessary
}




// int identifications
  float rightMult = 1;
  float leftMult = 1;
  float multEccenticity = 0.93;
  int shimmySpeed = 20;
  float globalCurrentRev = 0;

  bool shimmyON = false;

  float shimmyHead;
  float shimmyRange;

  bool isDebugging = false;

// end identifications


//Varibles
int pidTurnSpeed = 45;

//Intake ON tag4
// these functions just spin the intakes a certain direction or turn them off.
void Robot::intakeON()
{
  Intake1.spin(vex::directionType::rev, 200, vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::fwd, 200, vex::velocityUnits::rpm);
}

void Robot::intakeOUT()
{
  Intake1.spin(vex::directionType::fwd, 200, vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::rev, 200, vex::velocityUnits::rpm);
}

//Intake OFF
void Robot::intakeOFF()
{
  Intake1.stop();
  Intake2.stop();
}

//intake roll, which is spinning the intakes for a certain time before turning them off. 
void intake(float sec, int fastness)
{
  Intake1.spin(vex::directionType::rev, fastness, vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::fwd, fastness, vex::velocityUnits::rpm);
  vex::task::sleep(sec);
  Intake1.stop();
  Intake2.stop();
}


// other drive functions   tag5
void turnPidRad(float rotationValue, float rightTurnSpeed, float leftTurnSpeed, rotationUnit units = rotationUnit::rev, int count = 0, bool absoluteTurn = false, float timeout = 60)
{
  // turnpid rad is sort of like turnpid except for the small fact that turnpidrad turns around a certain point outside the bot. the bot is turning around a vertext, on the circumference. 


  float startDeg = inertialSensor.heading(degrees);
  startDeg = startDeg > 180 ? startDeg - 360 : startDeg;

  if (absoluteTurn)
  {
    rotationValue = rotationValue - startDeg;
    rotationValue = rotationValue > 180 ? rotationValue - 360 : rotationValue;
    rotationValue = rotationValue < -180 ? rotationValue + 360 : rotationValue;
  }

  float rev = rotationValue;
  switch (units)
  {
  case rotationUnit::rev:
    rev = rotationValue;
    break;
  case rotationUnit::degrees:
    rev = (rotationValue / 90) * 0.748; //unkt
    break;
  default:
    rev = rotationValue;
    break;
  }

  float adjTurn;

  float centre = (leftTurnSpeed + rightTurnSpeed) / 2;
  float leftDistanceModifier = (leftTurnSpeed / (leftTurnSpeed - centre));
  float rightDistanceModifier = (rightTurnSpeed / (centre - leftTurnSpeed));

  Timer.clear();
  FRDrive.startRotateFor(rev * rightDistanceModifier, vex::rotationUnits::rev, rightTurnSpeed, vex::velocityUnits::pct);
  FLDrive.startRotateFor(rev * leftDistanceModifier, vex::rotationUnits::rev, leftTurnSpeed, vex::velocityUnits::pct);
  BLDrive.startRotateFor(rev * leftDistanceModifier, vex::rotationUnits::rev, leftTurnSpeed, vex::velocityUnits::pct);
  BRDrive.startRotateFor(rev * rightDistanceModifier, vex::rotationUnits::rev, rightTurnSpeed, vex::velocityUnits::pct);
  while ((FLDrive.isSpinning() || FRDrive.isSpinning() || BLDrive.isSpinning() || BRDrive.isSpinning()) && Timer.time(vex::timeUnits::sec) < timeout)
  {
    vex::task::sleep(20);
  }

  vex::task::sleep(250); //was 100
  float currentDeg = inertialSensor.heading(degrees);
  currentDeg = currentDeg > 180 ? currentDeg - 360 : currentDeg;
  float turnValue = currentDeg - startDeg;
  float turnGoal = rotationValue; //if its degrees bro
  float turnAmount = turnGoal - turnValue;
  turnAmount = turnAmount > 180 ? turnAmount - 360 : turnAmount;
  turnAmount = turnAmount < -180 ? turnAmount + 360 : turnAmount;

  adjTurn = turnAmount;

  Brain.Screen.print(adjTurn);
  Brain.Screen.print(", ");

  if (fabs(adjTurn) > 5) //was 1.5
  {

    if (isDebugging)
    {
      printControllerValue(turnAmount);
    }

    if (count == 0)
    {
      Timer3.clear();
    }

    if (Timer3.time(vex::timeUnits::sec) > 1)
    {
      return;
    }

    if (count < 1) //count was 4
    {
      turnPid(adjTurn, 15, rotationUnit::degrees, count + 1); //was 15,then 10,then 15
    }
  }
  //add 1 each time it recurses so you can get out if necessary
}


void rightDrive(float rev, float speed) // spins the right motors
{

  FRDrive.startRotateFor(-rev, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm);
  BRDrive.startRotateFor(-rev, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm);
}

void leftDrive(float rev, float speed) // spins the left motors
{

  FLDrive.startRotateFor(rev, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm);
  BLDrive.startRotateFor(rev, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm);
}

void brakeON() // turns the braketype to hold
{
  FRDrive.setBrake(vex::brakeType::hold);
  FLDrive.setBrake(vex::brakeType::hold);
  BRDrive.setBrake(vex::brakeType::hold);
  BLDrive.setBrake(vex::brakeType::hold);
}

void brakeOFF() // turns the braketype to coast
{
  FRDrive.setBrake(vex::brakeType::coast);
  FLDrive.setBrake(vex::brakeType::coast);
  BRDrive.setBrake(vex::brakeType::coast);
  BLDrive.setBrake(vex::brakeType::coast);
}

void setPidIntake() // does the first one but for the intakes
{
  Intake1.setBrake(vex::brakeType::hold);
  Intake2.setBrake(vex::brakeType::hold);
}

void setPidOFFIntake() // does the latter but for the intakes
{
  Intake1.setBrake(vex::brakeType::coast);
  Intake2.setBrake(vex::brakeType::coast);
}

//speed sets//////
  int slow = 30;  //
  int fast = 70;  //
  int full = 100; //
  int safe = 50;  //
//////////////////


float m = 1.78; //turning constant, prev was 1.265 for exact without play or friction

// moveincrease functions tag6


void moveIncreaseREV(float distValue, float maxSpeed, distUnit units = distUnit::inches)
{
  FLDrive.resetRotation();
  FRDrive.resetRotation();
  BLDrive.resetRotation();
  BRDrive.resetRotation();

  vex::task::sleep(5);

  float speed;
  float currentRev = 0;
  
  float maxRev = conversionCode(distValue, units);
  float sign = (maxRev > 0 ? 1 : -1);
  rightMult = 1;
  leftMult = 1;

  float getToTop = 1.5;
  float getToZero = 3;
  float speedOffset = 15;

  Timer.clear();
  float timerNow = Timer.time(vex::timeUnits::sec) + 60;
  bool isTicked = false;
  while (fabs(currentRev) < fabs(maxRev))
  {
    currentRev = ((fabs(BLDrive.rotation(vex::rotationUnits::rev)) + fabs(FRDrive.rotation(vex::rotationUnits::rev)) + fabs(FLDrive.rotation(vex::rotationUnits::rev)) + fabs(BRDrive.rotation(vex::rotationUnits::rev))) / 4);
    globalCurrentRev = currentRev;
    speed = maxSpeed;

    if (currentRev < getToTop)
    {
      float timeSpeed = -(100 / 2) * cos((6.28318530718 / 3) * (Timer.time(vex::timeUnits::sec))) + (100 / 2);
      if (speed > timeSpeed)
      {
        speed = timeSpeed;
      }
    }
    else if (fabs(maxRev) - currentRev < getToZero)
    {
      float projectedCurrentRev = currentRev + 0.05;

      float maxSlopeSpeed = (0.5 * cos((6.28318530718 / (getToZero * 2)) * ((fabs(maxRev) - fabs(projectedCurrentRev)) + (getToZero))) + 0.5) * 100 + speedOffset;

      if (speed > maxSlopeSpeed)
      {
        speed = maxSlopeSpeed;
      }
    }

    float minSpeed = 3;

    if (speed < minSpeed)
    {
      speed = minSpeed;
    }

    BLDrive.spin(directionType::fwd, speed * sign * leftMult, velocityUnits::pct);
    FLDrive.spin(directionType::fwd, speed * sign * leftMult, velocityUnits::pct);
    BRDrive.spin(directionType::rev, speed * sign * rightMult, velocityUnits::pct);
    FRDrive.spin(directionType::rev, speed * sign * rightMult, velocityUnits::pct);

    while (FLDrive.isSpinning() || FRDrive.isSpinning() || BLDrive.isSpinning() || BRDrive.isSpinning())
    {

      if (!isTicked && (!FLDrive.isSpinning() || !FRDrive.isSpinning() || !BLDrive.isSpinning() || !BRDrive.isSpinning()))
      {

        isTicked = true;

        timerNow = Timer.time(vex::timeUnits::sec) + 0.1;
      }
      if (Timer.time(vex::timeUnits::sec) > timerNow)
      {
        break;
      }
      vex::task::sleep(5);
    }
  }
  BLDrive.stop();
  BRDrive.stop();
  FLDrive.stop();
  FRDrive.stop();
} 

void moveIncrease(float sec, float maxSpeed, float multiple = 1)
{
  Timer.clear();

  float speed;

  while (Timer.time(vex::timeUnits::sec) < sec)
  {
    speed = (-(maxSpeed / 2) * cos((6.28318530718 / sec) * (Timer.time(vex::timeUnits::sec))) + (maxSpeed / 2)) * multiple;
    if (speed > maxSpeed)
    {
      speed = maxSpeed;
    }
    BLDrive.spin(directionType::fwd, speed, velocityUnits::pct);
    FLDrive.spin(directionType::fwd, speed, velocityUnits::pct);
    BRDrive.spin(directionType::rev, speed, velocityUnits::pct);
    FRDrive.spin(directionType::rev, speed, velocityUnits::pct);
  }

  FRDrive.stop();
  FLDrive.stop();
  BLDrive.stop();
  BRDrive.stop();
}

void moveIncreaseRight() // drivesmooth but for the right side of the robot
{

  //________________________
  float sec = 0.5;
  float maxSpeed = 30;
  //________________________

  Timer.clear();

  float speed;

  while (Timer.time(vex::timeUnits::sec) < sec)
  {
    speed = -(maxSpeed / 2) * cos((6.28318530718 / sec) * (Timer.time(vex::timeUnits::sec))) + (maxSpeed / 2);
    BRDrive.spin(directionType::rev, speed, velocityUnits::pct);
    FRDrive.spin(directionType::rev, speed, velocityUnits::pct);
  }

  FRDrive.stop();
  BRDrive.stop();
  printController("right done");
}


void moveIncreaseLeft() // drivesmooth but for the left side of the robot
{
  //________________________
  float sec = 0.5;
  float maxSpeed = -30;
  //________________________
  Timer4.clear();

  float speed;

    speed = maxSpeed;
    
    BLDrive.spin(directionType::fwd, speed, velocityUnits::pct);
    FLDrive.spin(directionType::fwd, speed, velocityUnits::pct);

  while (Timer4.time(vex::timeUnits::sec) < sec)
  {
    vex::task::sleep(5);
  }

  FLDrive.stop();
  BLDrive.stop();
  printController("left done");
}

void moveIncreaseLeft3() // drivesmooth but for the left side of the robot
{
  //________________________
  float sec = 0.6;
  float maxSpeed = -30;
  //________________________
  Timer4.clear();

  float speed;

    speed = maxSpeed;
    
    BLDrive.spin(directionType::fwd, speed, velocityUnits::pct);
    FLDrive.spin(directionType::fwd, speed, velocityUnits::pct);

  while (Timer4.time(vex::timeUnits::sec) < sec)
  {
    vex::task::sleep(5);
  }

  FLDrive.stop();
  BLDrive.stop();
  printController("left done");
}


void moveIncreaseLeft2() // drivesmooth but for the left side of the robot
{
  //________________________
  float sec = 1.25;
  float maxSpeed = 30;
  //________________________
  Timer4.clear();

  float speed;

  while (Timer4.time(vex::timeUnits::sec) < sec)
  {
    speed = -(maxSpeed / 2) * cos((6.28318530718 / sec) * (Timer4.time(vex::timeUnits::sec))) + (maxSpeed / 2);
    BLDrive.spin(directionType::fwd, speed, velocityUnits::pct);
    FLDrive.spin(directionType::fwd, speed, velocityUnits::pct);
  }

  FLDrive.stop();
  BLDrive.stop();
  printController("left done");

}




//intake task vars
int intakeTaskSpeed = -50;
int intakeTaskMsec = 330;

float threshold = 69.4;

int stopIfIndexed() // uses the light sensor to detect the light level when intaking. If there is an object covering the sensor, the light level will decrease, 
// therefore signaling us that we have an object indexed (AKA past the intakes). when the light sensor detects the light level below a certain threshold, it stops intaking and continues 
// with the code.
{

  
  float b = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
  vex::task::sleep(8);
  float c = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
  vex::task::sleep(8);
  float d = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);

  Brain.Screen.print(b);

  while (1)
  {
    Intake1.spin(vex::directionType::rev, intakeTaskSpeed, vex::velocityUnits::rpm);
    Intake2.spin(vex::directionType::fwd, intakeTaskSpeed, vex::velocityUnits::rpm);

    b = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
    vex::task::sleep(8);
    c = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
    vex::task::sleep(8);
    d = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
    if (b < threshold && c < threshold && d < threshold)
    {
      break;
    }
  }
  Intake1.stop();
  Intake2.stop();
  vex::task::sleep(200);

  Intake1.spin(vex::directionType::rev, intakeTaskSpeed, vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::fwd, intakeTaskSpeed, vex::velocityUnits::rpm);
  vex::task::sleep(intakeTaskMsec);
  Intake1.stop();
  Intake2.stop();
  return 0;
}

int intakeTaskMsec2 = 85;

int stopIfIndexed2()
{

  
  float b = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
  vex::task::sleep(8);
  float c = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
  vex::task::sleep(8);
  float d = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);

  Brain.Screen.print(b);

  while (1)
  {
    Intake1.spin(vex::directionType::rev, intakeTaskSpeed, vex::velocityUnits::rpm);
    Intake2.spin(vex::directionType::fwd, intakeTaskSpeed, vex::velocityUnits::rpm);

    b = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
    vex::task::sleep(8);
    c = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
    vex::task::sleep(8);
    d = vex::analog_in(Brain.ThreeWirePort.H).value(vex::percentUnits::pct);
    if (b < threshold && c < threshold && d < threshold)
    {
      break;
    }
  }
  Intake1.stop();
  Intake2.stop();
  vex::task::sleep(200);

  Intake1.spin(vex::directionType::rev, intakeTaskSpeed, vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::fwd, intakeTaskSpeed, vex::velocityUnits::rpm);
  vex::task::sleep(intakeTaskMsec2);
  Intake1.stop();
  Intake2.stop();
  return 0;
}

int autonstoptask()
{

  vex::task::sleep(15000);
  Intake1.setBrake(vex::brakeType::coast);
  Intake2.setBrake(vex::brakeType::coast);
  BRDrive.setBrake(vex::brakeType::coast);
  BLDrive.setBrake(vex::brakeType::coast);
  FRDrive.setBrake(vex::brakeType::coast);
  FLDrive.setBrake(vex::brakeType::coast);
  while (true)
  {
    Intake1.stop();
    Intake2.stop();
    FRDrive.stop();
    FLDrive.stop();
    BLDrive.stop();
    BRDrive.stop();
  }

  return 0;
}

void autonstoptaskFUNCTION()
{
  //calls the auton stop task above
  vex::task t(autonstoptask);
}

void stopIfIndexedFUNCTION()
{
  vex::task t(stopIfIndexed);
}

int intake() // the autonomous function that signals the intake to run.
{
  Intake1.spin(vex::directionType::rev, intakeTaskSpeed, vex::velocityUnits::rpm);
  Intake2.spin(vex::directionType::fwd, intakeTaskSpeed, vex::velocityUnits::rpm);
  vex::task::sleep(intakeTaskMsec);
  Intake1.stop();
  Intake2.stop();
  return 0;
}

void callIntakeTask()
{
  vex::task t(intake);
}



// tag8

// uses the rangefinder sensor to detect a distance from the wall, then subtracts that distance from where you would like to end up so it becomes absolute.
// rangefinder sensors in the back so we used to ultrasonic sensors in the back of a robot to measure the distance from the robot to the wall this
// helps us move more accurately as we're not just measuring the encoders on the V5 base motors but we're actually measuring the distance from the robot to the wall and we used to 
// position sensors and take the average as it's more accurate than just taking one 
// and we use this and this certain section of our programming skills when we're backing up from point A to point B and it's more consistent if we use ultrasonic sensors


void moveMinusRangefinderIN(float distValue, float speed, bool turnPidON = false, bool turnPidOFF = false) 
{ 
  if (turnPidON)
  {
    brakeON();
  }
  float rangeDist = RangeFinderA.distance(vex::distanceUnits::in); // this line reads the rangefinder distance.
  float go = distValue - rangeDist + 1; // this line calculates the variable go.

  moveIncreaseREV(go, speed, distUnit::inches); // this line tells the robot to move 'go' amount
  if (turnPidOFF)
  {
    brakeOFF();
  }
}


//user control
void Robot::Usercontrol(){ 

 
  while (1) {  
    //Drive
    // ARCADE CONTROL
    BLDrive.spin(directionType::fwd, -Controller.Axis3.value() - (Controller.Axis1.value() * 0.5), velocityUnits::pct);
    FLDrive.spin(directionType::fwd, -Controller.Axis3.value() - (Controller.Axis1.value() * 0.5), velocityUnits::pct);
    BRDrive.spin(directionType::rev, -Controller.Axis3.value() + (Controller.Axis1.value() * 0.5), velocityUnits::pct);
    FRDrive.spin(directionType::rev, -Controller.Axis3.value() + (Controller.Axis1.value() * 0.5), velocityUnits::pct);

    // INTAKES
    if (Controller.ButtonL2.pressing())
    {
        Intake1.spin(vex::directionType::fwd, 200, vex::velocityUnits::rpm);
        Intake2.spin(vex::directionType::rev, 200, vex::velocityUnits::rpm);

    }
    else if (Controller.ButtonL1.pressing())
    {
      Intake1.spin(vex::directionType::rev, 200, vex::velocityUnits::rpm);
      Intake2.spin(vex::directionType::fwd, 200, vex::velocityUnits::rpm);

    }
    else
    {
        Intake1.stop();
        Intake2.stop();

    }




    // ROLLER 

    if (Controller.ButtonR2.pressing()) // in
    {
      Roller1.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      Roller2.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);

    }
    else if (Controller.ButtonR1.pressing()) // out
    {
      Roller1.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      Roller2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

    }
    else
    {
      Roller1.stop();
      Roller2.stop();

    }

  }

}

void Robot::rollermove (float sec, int fastness)
{
  Roller1.spin(vex::directionType::rev, fastness, vex::velocityUnits::rpm);
  Roller2.spin(vex::directionType::fwd, fastness, vex::velocityUnits::rpm);
  vex::task::sleep(sec*1000);
  Roller1.stop();
  Roller2.stop();
}

void Robot::backupslam (float sec, int fastness)
{
  brakeOFF();
  BLDrive.spin(vex::directionType::rev, fastness, vex::velocityUnits::rpm);
  FLDrive.spin(vex::directionType::rev, fastness, vex::velocityUnits::rpm);
  BRDrive.spin(vex::directionType::fwd, fastness, vex::velocityUnits::rpm);
  FRDrive.spin(vex::directionType::fwd, fastness, vex::velocityUnits::rpm);
  vex::task::sleep(sec*1000);
  BLDrive.stop();
  FLDrive.stop();
  BRDrive.stop();
  FRDrive.stop();
  brakeON();

}


void Robot::rangemove (float distValue, float speed, bool turnPidON, bool turnPidOFF) // uses the rangefinder sensor to detect a distance from the wall, then subtracts that distance from where you would like to end up so it becomes absolute.
// rangefinder sensors in the back so we used to ultrasonic sensors in the back of a robot to measure the distance from the robot to the wall this
// helps us move more accurately as we're not just measuring the encoders on the V5 base motors but we're actually measuring the distance from the robot to the wall and we used to 
// position sensors and take the average as it's more accurate than just taking one 
// and we use this and this certain section of our programming skills when we're backing up from point A to point B and it's more consistent if we use ultrasonic sensors
{ 
  if (turnPidON)
  {
    brakeON();
  }
  float rangeDist1 = RangeFinderA.distance(vex::distanceUnits::in);
  float rangeDist2 = RangeFinderC.distance(vex::distanceUnits::in);
  float rangeDist = (rangeDist1 + rangeDist2) / 2;
  float go = distValue - rangeDist;

  moveIncreaseREV(go, speed, distUnit::inches);

  if (turnPidOFF)
  {
    brakeOFF();
  }
}


void Robot::wait(float sec){
  vex::task::sleep(sec*1000);
}


void Robot::runmotors(){
  float speed = 80;
  FRDrive.startRotateFor(-5, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm);
  BRDrive.startRotateFor(-5, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm);
  FLDrive.startRotateFor(5, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm);
  BLDrive.startRotateFor(5, vex::rotationUnits::rev, speed, vex::velocityUnits::rpm);
  vex::task::sleep(3000);
}

void Robot::inertialcalibrate()
{
  inertialSensor.calibrate();
  echo("Calibrating...");
  while (inertialSensor.isCalibrating()) 
  {
    vex::task::sleep(20);
  }

  echo("Calibrated!");
}


//intake task vars
int rollerTaskSpeed = -50;
int rollerTaskMsec = 330; //was 360

float thresHold = 69.4;

int stopifready() // uses the light sensor to detect the light level when intaking. If there is an object covering the sensor, the light level will decrease, 
// therefore signaling us that we have an object indexed (AKA past the intakes). when the light sensor detects the light level below a certain threshold, it stops intaking and continues 
// with the code.
{

  
  float b = vex::analog_in(Brain.ThreeWirePort.B).value(vex::percentUnits::pct);
  vex::task::sleep(8);
  float c = vex::analog_in(Brain.ThreeWirePort.B).value(vex::percentUnits::pct);
  vex::task::sleep(8);
  float d = vex::analog_in(Brain.ThreeWirePort.B).value(vex::percentUnits::pct);

  Brain.Screen.print(b);

  while (1)
  {
    Roller1.spin(vex::directionType::rev, rollerTaskSpeed, vex::velocityUnits::rpm);
    Roller2.spin(vex::directionType::fwd, rollerTaskSpeed, vex::velocityUnits::rpm);

    b = vex::analog_in(Brain.ThreeWirePort.B).value(vex::percentUnits::pct);
    vex::task::sleep(8);
    c = vex::analog_in(Brain.ThreeWirePort.B).value(vex::percentUnits::pct);
    vex::task::sleep(8);
    d = vex::analog_in(Brain.ThreeWirePort.B).value(vex::percentUnits::pct);
    if (b < thresHold && c < thresHold && d < thresHold)
    {
      break;
    }
  }
  Roller1.stop();
  Roller2.stop();
  vex::task::sleep(200);

  Roller1.spin(vex::directionType::rev, rollerTaskSpeed, vex::velocityUnits::rpm);
  Roller2.spin(vex::directionType::fwd, rollerTaskSpeed, vex::velocityUnits::rpm);
  vex::task::sleep(rollerTaskMsec);
  Roller1.stop();
  Roller2.stop();
  return 0;
}

void stopifreaddy()
{
  vex::task t(stopifready);
}


//______________________________________________________________________________________________________________________________________________________________________//
//                                                                                                                                                                      //
//                                                                                                                                                                      //
//                                                                             Skills  tag7                                                                                 //
//                                                                                                                                                                      //
//                                                                                                                                                                      //
//______________________________________________________________________________________________________________________________________________________________________//

void Robot::skillz(){

  //Line up with first goal
  float sped = 30;
  moveIncreaseLeft();
   echo("1");
  
  //Score first goal
  rollermove(3, 100);
   echo("2");
  moveIncreaseLeft2();
  turnPid(90, 30, rotationUnit::degrees, 0, true);
   echo("3");
  driveSmooth(-17, sped, distUnit::inches);
   echo("4");
  turnPid(-90, 30, rotationUnit::degrees);
   echo("5");
  backupslam(1.5, -30);
   echo("6");
  intakeON();
   echo("7");

  //Drive to second ball
  driveSmooth(2.05, sped, distUnit::tiles);
  wait(1);
  rollermove(0.3, 100);
  driveSmooth(-0.20, sped, distUnit::tiles);
  intakeOFF();
   echo("9");
  turnPid(-90, 30, rotationUnit::degrees);
   echo("10");
  intakeOUT();
  driveSmooth(0.55, sped, distUnit::tiles);
   echo("11");
  intakeOFF();
  driveSmooth(-2, sped, distUnit::inches);
   echo("12");
  moveIncreaseLeft();
  brakeOFF();
  driveSmooth(0.3, sped, distUnit::inches);
  brakeON();
   echo("13");
  //Score middle goal
  rollermove(3, 80);
   echo("14");

  //Head towards ball
  driveSmooth(-5, 30, distUnit::inches);
  turnPid(-90, 30, rotationUnit::degrees, 0, true);
   echo("15");
  driveSmooth(-0.55, 30, distUnit::tiles);
   echo("16");
  turnPid(0, 30, rotationUnit::degrees, 0, true);
   echo("17");
  intakeON();
  driveSmooth(1.8, 30, distUnit::tiles);
   echo("18");
  wait(1);
  rollermove(0.17, 100);
  intakeOFF();
  driveSmooth(-0.05, 30, distUnit::tiles);
   echo("19");
  turnPid(-90, 30, rotationUnit::degrees);
   echo("21");
  intakeON();
  moveMinusRangefinderIN(-65.8, 30);
   echo("22");
  wait(1);
  rollermove(0.17, 100);
  intakeOFF();
   echo("23");
  turnPid(-45, 30, rotationUnit::degrees, 0, true);
  driveSmooth(7, 30, distUnit::inches);
   echo("24");
  rollermove(2, 100);

}


