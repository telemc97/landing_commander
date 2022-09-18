#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_core/GridMap.hpp>

namespace landing_commander{

class LandingCommander{

  public:
  
  LandingCommander(const ros::NodeHandle &nh_ = ros::NodeHandle()); //constructor
  virtual ~LandingCommander(); //destructor

  void gridHandler(const nav_msgs::OccupancyGrid::ConstPtr& gridMap);


  protected:

  bool splitncheck(const Eigen::MatrixXf& mapdata, Eigen::Array2i& robotIndex, double resolution, double minLandA, Eigen::MatrixX3i& land_waypoints, int offset_w = 0, int offest_h = 0);
  
  inline int maxDivider(int number){
    int divider;
    for (int i=2; i<number; i++){
      if (number%i==0){
        divider = i;
        break;
      }
      if (divider == number){
        divider = 1;
        break;
      }
    }
    return divider;
  }

  tf::TransformListener tfListener;

  std::string baseFrameId;

  ros::NodeHandle nodeHandle;
  message_filters::Subscriber<nav_msgs::OccupancyGrid>* gridMapSub;
  tf::MessageFilter<nav_msgs::OccupancyGrid>* tfgridMapSub;
  ros::Publisher occupancySub;
  // geometry_msgs::Point* point;
  grid_map::GridMap gridMapConverted;
  nav_msgs::OccupancyGrid gridMapOutput;
  double minimumLandingArea;
  // Eigen::MatrixXi l_waypoints;

  float safetyRadius;

  bool latchedTopics;
};
}