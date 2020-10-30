// Function Identifications


enum class distUnit
{
  rev,
  inches,
  feet,
  tiles
};
enum class rotationUnit
{
  rev,
  degrees
};
enum class positionUnit
{
  relative,
  absolute
};



class Robot 
{
  public:
    void driveSmooth(float distValue, float maxSpeed, distUnit units = distUnit::inches);
    void echo(const char *message);
    void echonum(float);
    void turnPid(float rotationValue, float TurnSpeed, rotationUnit units = rotationUnit::rev, int count = 0, bool absoluteTurn = false, float timeout = 60);
    void Visiontask();
    void CentreGreen();
    void takeSnap();
    int getDistance();
    void brakeAuto();
    void Usercontrol();
    void CALLmoveIncreaseLeft();
    void CALLmoveIncreaseLeft2();
    void CALLmoveIncreaseRight();
    void rollermove(float sec, int fastness);
    void intakeON();
    void intakeOUT();
    void intakeOFF();
    void backupslam(float sec, int fastness);
    void rangemove (float distValue, float speed, bool turnPidON = false, bool turnPidOFF = false);
    void wait (float sec);
    void runmotors();
    void lightSensorRollStop();
    void inertialcalibrate();
    void skillz();

};

class senseBot
{
  public:
    void getToHere(int xTile, int yTile, float speed);

};