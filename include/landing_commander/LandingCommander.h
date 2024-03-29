#include <ros/ros.h>
#include <string>
#include <chrono>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <landing_commander/LandingCommanderDebug.h>
#include <landing_commander/LandingTargets.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Transform.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>


#include <nav_msgs/OccupancyGrid.h>

namespace landing_commander{

class LandingCommander{

  public:

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, mavros_msgs::State, mavros_msgs::ExtendedState> MySyncPolicy;
  
  LandingCommander(const ros::NodeHandle &nh_ = ros::NodeHandle()); //constructor
  virtual ~LandingCommander(); //destructor

  void mainCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridMap, const mavros_msgs::State::ConstPtr& state, const mavros_msgs::ExtendedState::ConstPtr& extendedState);

  protected:

  void toMatrix(const nav_msgs::OccupancyGrid& occupancyGrid, Eigen::MatrixXi& matrix, double& ratio_);
  
  void toOccupancyGrid(const Eigen::MatrixXi& matrix, nav_msgs::OccupancyGrid& occupancyGrid, const nav_msgs::MapMetaData& mapMetaData, const std_msgs::Header& header);
  
  void splincheckStride(const Eigen::MatrixXi& matrix, const geometry_msgs::Pose& origin, Eigen::MatrixX3i& land_waypoints, const double& ratio, const double& stride_coef_, const double& targetProcTime_, const Eigen::Array2i& robotIndex, const int& safetyRadius);

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

  void checkEmMarkEm(Eigen::MatrixXi& matrix, const int& radius);

  void Debug( Eigen::MatrixXi& matrix, const Eigen::MatrixX3i& land_waypoints, const geometry_msgs::Pose& origin);

  bool checkArea(Eigen::MatrixXi& matrix_){
    bool occupied_ = false;
    for (int i=0;i<matrix_.rows();i++){
      for (int j=0;j<matrix_.cols();j++){
        if (matrix_(i,j)==100){
          occupied_ = true;
        }
      }
    }
    return occupied_;
  }

  Eigen::MatrixX3i sortAscOrder (Eigen::MatrixX3i& land_waypoints){
    Eigen::Matrix<int,1,3> temp;
    for (int i=0;i<land_waypoints.rows();i++){
      for (int j=i+1;j<land_waypoints.rows();j++){
        if (land_waypoints(i,2)>land_waypoints(j,2)){
          temp = land_waypoints.block(i,0,1,3);
          land_waypoints.block(i,0,1,3) = land_waypoints.block(j,0,1,3);
          land_waypoints.block(j,0,1,3) = temp;
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

  int getLinearIndexFromIndex(int& height, int& width, Eigen::Array2i& Array){
    int LinearIndex;
    int x = Array(0);
    int y = Array(1);
    LinearIndex = x*height + y;
    return LinearIndex;
  }

  void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
  }

  void commander(const ros::TimerEvent&);

  void safety(const ros::TimerEvent&);

  void checkLandingPoint();

  void fallbackSafety();

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

  Eigen::Matrix<int,1,3> fromMatrixToRealWorld(const Eigen::Matrix<int,1,3>& point, const geometry_msgs::Pose& origin){
    Eigen::Matrix<int,1,3> rw_point;
    rw_point(0) = point(0)+origin.position.x;
    rw_point(1) = point(1)+origin.position.y;
    rw_point(2) = point(2);

    return rw_point;
  }

  Eigen::Matrix<int,1,3> fromRealWorldToMatrix(const Eigen::Matrix<int,1,3>& rw_point, const geometry_msgs::Pose& origin){
    Eigen::Matrix<int,1,3> point;
    point(0) = rw_point(0)-origin.position.x;
    point(1) = rw_point(1)-origin.position.y;
    point(2) = rw_point(2);

    return point;
  }

  int euclidianDistance(const Eigen::Matrix<int,1,2>& pt0, const Eigen::Matrix<int,1,2>& pt1){
    double dist;
    dist = std::sqrt(std::pow((pt1(0)-pt0(0)),2)+std::pow((pt1(1)-pt0(1)),2));
    int dist_int = std::round(dist);
    return dist;
  }

  bool waypointReached(const Eigen::Matrix<int,1,2>& robot_pose, const geometry_msgs::Point& land_target){
    bool waypointReached;
    if (robot_pose(0)==land_target.x && robot_pose(1)==land_target.y){
      waypointReached=true;
    }else{
      waypointReached=false;
    }
    return waypointReached;
  }

  bool validPoint(const Eigen::Matrix<int,1,3>& land_point_, const Eigen::MatrixX3i land_points_){
    bool isValid = false;
    for (int i=0;i<land_points_.rows();i++){
      if (land_points_(i,0) == land_point_(0) && land_points_(i,1)==land_point_(1)){
        isValid = true;
      }
    }
    return isValid;
  }

  mavros_msgs::State current_state;
  tf::TransformListener tfListener;

  std::string baseFrameId;

  ros::NodeHandle nodeHandle;
  message_filters::Subscriber<nav_msgs::OccupancyGrid>* gridMapSub;
  message_filters::Subscriber<geometry_msgs::PoseStamped>* postitionSub;
  message_filters::Subscriber<mavros_msgs::State>* fcuStateSub;
  message_filters::Subscriber<mavros_msgs::ExtendedState>* fcuExtendedStateSub;

  Eigen::MatrixX3i land_points;

  //Commander loop
  Eigen::Matrix<int,1,3> active_land_point;
  bool land_point_serching;
  geometry_msgs::Pose map_origin;
  bool armed;
  std::string mode;

  tf::MessageFilter<nav_msgs::OccupancyGrid>* tfgridMapSub;
  message_filters::Synchronizer<MySyncPolicy>* sync;
  

  ros::Publisher occupancyPub;
  ros::Publisher pos_setpoint;
  ros::Publisher debugger;
  ros::Publisher debugger_landingPoints;

  ros::Timer commanderTimer;
  ros::Timer guardTimer;
  ros::Timer checkingTimer;
  landing_commander::LandingCommanderDebug debug_msg;
  landing_commander::LandingTargets landingTargets_msg;
  landing_commander::LandingTarget landingTarget_msg;
  nav_msgs::OccupancyGrid gridMapOutput;
  Eigen::MatrixXi OccupancyGridEigen;
  Eigen::MatrixXi distanceMatrix;
  Eigen::Matrix<int,1,2> robotPose;

  ros::ServiceClient arming_client;
  ros::ServiceClient set_mode_client;


  int landState;
  double safetyRadius;
  double ratio;

  double minStride;

  double targetProcTime;
  double stride_coef;

  bool latchedTopics;
  bool enableGuard;
  bool land2base;
  bool debug;
  bool publishOccupancy;
  bool safetyArea;
  bool haveOccupancyGridEigen=false;

  //commander stuff
  geometry_msgs::PoseStamped land_pose;
  mavros_msgs::SetMode land_set_mode;
  mavros_msgs::SetMode position_set_mode;
  mavros_msgs::SetMode offboard_set_mode;
  int land_pose_dist;
  Eigen::MatrixX3i land_points_temp;
};
}