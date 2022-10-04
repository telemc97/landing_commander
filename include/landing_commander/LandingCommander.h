#include <ros/ros.h>
#include <string>
#include <chrono>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Transform.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>


#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_core/GridMap.hpp>

namespace landing_commander{

class LandingCommander{

  public:

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, mavros_msgs::State, mavros_msgs::ExtendedState> MySyncPolicy;
  
  LandingCommander(const ros::NodeHandle &nh_ = ros::NodeHandle()); //constructor
  virtual ~LandingCommander(); //destructor

  void mainCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridMap, const mavros_msgs::State::ConstPtr& state, const mavros_msgs::ExtendedState::ConstPtr& extendedState);

  protected:

  bool toMatrix(const nav_msgs::OccupancyGrid& occupancyGrid, Eigen::MatrixXi& matrix);
  
  bool toOccupancyGrid(const Eigen::MatrixXi& matrix, nav_msgs::OccupancyGrid& occupancyGrid, const nav_msgs::MapMetaData& mapMetaData, const std_msgs::Header& header);

  bool splitncheckExpirimental(const Eigen::MatrixXi& matrix, Eigen::Array2i& robotIndex, double resolution, double minLandA, Eigen::MatrixX4i& land_waypoints, bool& land2base, int offset_w = 0, int offest_h = 0);
  
  void splincheckStride(const Eigen::MatrixXi& matrix, Eigen::MatrixX4i& land_waypoints, const int& stride, const Eigen::Array2i& robotIndex, const int& safetyRadius);

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

  void checkEmMarkEm(Eigen::MatrixXi& matrix, int& radius);

  void Debug( Eigen::MatrixXi& matrix, const Eigen::MatrixX4i& land_waypoints);

  bool checkArea(Eigen::MatrixXi& matrix){
    bool occupied_ = false;
    for (int i=0;i<matrix.rows();i++){
      for (int j=0;j<matrix.cols();j++){
        if (matrix(i,j)==100){
          occupied_ = true;
        }
      }
    }
    return occupied_;
  }
  
  Eigen::MatrixX4i sortDescOrder (Eigen::MatrixX4i& land_waypoints, const int& col){
    Eigen::Matrix<int,1,4> temp;
    for (int i=0;i<land_waypoints.rows();i++){
      for (int j=i+1;j<land_waypoints.rows();j++){
        if (land_waypoints(i,col)>land_waypoints(j,col)){
          temp = land_waypoints.block(i,0,1,4);
          land_waypoints.block(i,0,1,4) = land_waypoints.block(j,0,1,4);
          land_waypoints.block(j,0,1,4) = temp;
        }
      }
    }
    return land_waypoints;
  }

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

  void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
  }

  void handleWaypoint(const ros::TimerEvent&);

  int metersToGrids(const double& meters, const double& res){
    int grids;
    grids = meters/res;
    if (grids<1){grids=1;}
    return grids;
  }

  double gridsToMeters(const int& grids, const double& res){
    double meters;
    meters = grids*res;
    return meters;
  }

  inline int maxDivider(int number){
    int divider;
    for (int i=2; i<number+1; i++){
      if (number%i==0){
        divider = i;
        break;
      }
    }
    if (divider == number){
      divider = 1;
    }
    return divider;
  }

  bool waypointReached(const Eigen::Array2i& robot_pose, const Eigen::Array2i& land_target_){
    bool waypointReached;
    if (robot_pose(0)==land_target_(0)&&robot_pose(1)==land_target_(1)){
      waypointReached=true;
    }else{
      waypointReached=false;
    }
    return waypointReached;
  }

  mavros_msgs::State current_state;
  tf::TransformListener tfListener;

  std::string baseFrameId;

  ros::NodeHandle nodeHandle;
  message_filters::Subscriber<nav_msgs::OccupancyGrid>* gridMapSub;
  message_filters::Subscriber<geometry_msgs::PoseStamped>* postitionSub;
  message_filters::Subscriber<mavros_msgs::State>* fcuStateSub;
  message_filters::Subscriber<mavros_msgs::ExtendedState>* fcuExtendedStateSub;

  Eigen::MatrixX4i land_points;

  tf::MessageFilter<nav_msgs::OccupancyGrid>* tfgridMapSub;
  message_filters::Synchronizer<MySyncPolicy>* sync;
  

  ros::Publisher occupancyPub;
  ros::Publisher pos_setpoint;
  ros::Timer landingHandler;
  nav_msgs::OccupancyGrid gridMapOutput;
  Eigen::MatrixXi OccupancyGridEigen;
  Eigen::Array2i robotPose;

  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;


  int landState;
  int stride;
  bool armed;
  std::string mode;
  double safetyRadius;

  bool latchedTopics;
  bool land2base;
  bool debug;
  bool publishOccupancy;
  bool safetyArea;
  bool haveOccupancyGridEigen;
};
}