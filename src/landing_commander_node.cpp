#include <ros/ros.h>
#include <landing_commander/LandingCommander.h>

using namespace landing_commander;

int main(int argc, char** argv){
  ros::init(argc, argv, "landing_commander");
  const ros::NodeHandle nh;

  LandingCommander server(nh);

  ros::spin();
  
  // ros::MultiThreadedSpinner spinner(4);
  // spinner.spin();

  return 0;
}