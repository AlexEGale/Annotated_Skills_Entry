# include "Robot.h"
// # include "vex.h" // pull in when needed

class bot: public Robot{
  public:
    void left1();
    void left2();
    void left3();

    void right1();
    void right2();
    void right3();

    void leftAll();

    void rightAll();
    void SKILLZ();

    void test();
    void usercontrolled();
    void wahyt();
    void eko(const char *message);
    void Calibrate_Inertial_Sensor();
};