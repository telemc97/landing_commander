#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Transform.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_core/GridMap.hpp>

namespace landing_commander{

class LandingCommander{

  public:
  
  LandingCommander(const ros::NodeHandle &nh_ = ros::NodeHandle()); //constructor
  virtual ~LandingCommander(); //destructor

  void gridHandler(const nav_msgs::OccupancyGrid::ConstPtr& gridMap);


  protected:

  bool toMatrix(const nav_msgs::OccupancyGrid& occupancyGrid, Eigen::MatrixXi& gridEigen);
  
  bool toOccupancyGrid(const Eigen::MatrixXi& gridEigen, nav_msgs::OccupancyGrid& occupancyGrid, const nav_msgs::MapMetaData& mapMetaData, const std_msgs::Header& header);

  bool splitncheck(const Eigen::MatrixXi& mapdata, Eigen::Array2i& robotIndex, double resolution, double minLandA, Eigen::MatrixX4i& land_waypoints, bool& land2base, int offset_w = 0, int offest_h = 0);
  
  bool isIn(Eigen::MatrixX2i& matrix, const int& x, const int& y){
    bool isIn;
    for (int i=0;i<matrix.rows();i++){
      if (matrix(i,0)==x && matrix(i,1)==y){
        isIn = true;
      }else{
        isIn = false;
      }
    }
    return isIn;
  }

  Eigen::MatrixXi checkEmMarkEm(const Eigen::MatrixXi& matrix, int& radius);

  Eigen::MatrixXi Debug( Eigen::MatrixXi& matrix, Eigen::MatrixX4i& land_waypoints);

  Eigen::Array2i getIndexFromLinearIndex(int& rows, int& Index){
    Eigen::Array2i IndexXY;
    int y = Index/rows;
    int x = Index-(y*rows);
    IndexXY(0) = x; IndexXY(1) = y;
    return IndexXY;
  }

  int getLinearIndexFromIndex(int height, int width, Eigen::Array2i& Array){
    int LinearIndex;
    int x = Array(0);
    int y = Array(1);
    LinearIndex = x*height + y;
    return LinearIndex;
  }

  int checkNeighborsRadius (const double& safeAreaRadius, const double& resolution);

  void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
  }

  void handleLand(const Eigen::MatrixX4i& land_waypoints);

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

  mavros_msgs::State current_state;
  tf::TransformListener tfListener;

  std::string baseFrameId;

  ros::NodeHandle nodeHandle;
  message_filters::Subscriber<nav_msgs::OccupancyGrid>* gridMapSub;
  message_filters::Subscriber<geometry_msgs::PoseStamped>* postitionSub;
  
  Eigen::MatrixX4i* land_points;

  tf::MessageFilter<nav_msgs::OccupancyGrid>* tfgridMapSub;
  ros::Publisher occupancyPub;
  ros::Publisher pos_setpoint;
  ros::Subscriber mavros_state;
  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;
  Eigen::MatrixXi OccupancyGridEigen;
  nav_msgs::OccupancyGrid gridMapOutput;

  double minimumLandingArea;

  double safetyRadius;

  bool latchedTopics;
  bool land2base;
  bool debug;
  bool publishOccupancy;
};
}