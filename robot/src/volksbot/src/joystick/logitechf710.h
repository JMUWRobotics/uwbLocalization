#ifndef LOGITECHF170_HH
#define LOGITECHF170_HH

#include "joystick.h"
#include <ros/ros.h>
#include "volksbot/vels.h"
#include <std_msgs/String.h>

// non commented version for controller on mode-switch X, other mode-switch D, not to confuse with mode button
#define BUTTON_A        0x00 //0x01
#define BUTTON_B        0x01 //0x02
#define BUTTON_X        0x02 //0x00
#define BUTTON_Y        0x03 //0x03
#define BUTTON_LEFT     0x04 //0x04
#define BUTTON_RIGHT    0x05 //0x05
#define START           0x07 //0x09
#define LOGITECH        0x08 //0x0A
#define BACK            0x06 //0x08

#define THROTTLE_LEFT     0x02 //0x06
#define THROTTLE_RIGHT    0x05 //0x07

#define LSTICK_LEFTRIGHT  0x00 //0x00
#define LSTICK_UPDOWN     0x01 //0x01
#define RSTICK_LEFTRIGHT  0x03 //0x02
#define RSTICK_UPDOWN     0x04 //0x03
#define HUD_LEFTRIGHT     0x06 //0x04
#define HUD_UPDOWN        0x07 //0x05

#define STICK_MIN_ACTIVITY   1000

class LogitechF : public Joystick {
  public:

    LogitechF() : Joystick() { init(); };
    LogitechF(const char* fn) : Joystick(fn) { init(); };

    virtual void handleButton(uint8_t number, bool pressed, uint32_t time);
    virtual void handleAxis(uint8_t number, int16_t value, uint32_t time);

    inline double getLeftVel() {return leftvel;}
    inline double getRightVel() {return rightvel;}

  private:
    ros::NodeHandle n;
    ros::Publisher publisher;


    void sendSpeed();

    inline void init() {
      speed = 20.0;
      rightvel = leftvel = 0.0;
      publisher = n.advertise<volksbot::vels>("Vel", 100);
    }

    double leftvel, rightvel;
    double speed;

};

#endif
