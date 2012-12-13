#include "falkor_drivers/pwm_switch.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "PwmSwitch");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  PwmSwitch pwm_switch(nh, nh_private);
  ros::spin();
  return 0;
}
