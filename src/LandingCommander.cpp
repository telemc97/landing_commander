#include <landing_commander/LandingCommander.h>

namespace landing_commander{

LandingCommander::LandingCommander(const ros::NodeHandle &nh_)
: nodeHandle(nh_),
  gridMapSub(NULL),
  tfgridMapSub(NULL),
  fcuStateSub(NULL),
  fcuExtendedStateSub(NULL),
  sync(NULL),
  baseFrameId("base_link"),
  safetyRadius(2.0),
  stride(3),
  OccupancyGridEigen(0,0),
  latchedTopics(true),
  land2base(false),
  land_points(0,4),
  safetyArea(true),
  debug(true),
  publishOccupancy(true),
  haveOccupancyGridEigen(false)
  {
    nodeHandle.param("safety_dist", safetyRadius, safetyRadius);
    nodeHandle.param("stride", stride, stride);
    nodeHandle.param("robot_frame", baseFrameId, baseFrameId);
    nodeHandle.param("land_to_base", land2base, land2base);
    nodeHandle.param("debug",debug,debug);
    nodeHandle.param("show_occupancy_map", publishOccupancy,publishOccupancy);
    nodeHandle.param("mark_safety_area",safetyArea, safetyArea);

    occupancyPub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("occupancy_output", 5, latchedTopics);
    pos_setpoint = nodeHandle.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    landingHandler = nodeHandle.createTimer(ros::Rate(20), &LandingCommander::handleWaypoint, this);

    set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");



    gridMapSub = new message_filters::Subscriber<nav_msgs::OccupancyGrid>(nodeHandle, "/projected_map", 10);
    tfgridMapSub = new tf::MessageFilter<nav_msgs::OccupancyGrid>(*gridMapSub, tfListener, baseFrameId, 10);
    fcuStateSub = new message_filters::Subscriber<mavros_msgs::State>(nodeHandle, "/mavros/state", 10);
    fcuExtendedStateSub = new message_filters::Subscriber<mavros_msgs::ExtendedState>(nodeHandle, "/mavros/extended_state", 10);

    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *tfgridMapSub, *fcuStateSub, *fcuExtendedStateSub);

    sync->registerCallback(boost::bind(&LandingCommander::mainCallback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));
    
    // tfgridMapSub->registerCallback(boost::bind(&LandingCommander::mainCallback, this, boost::placeholders::_1));
  }

LandingCommander::~LandingCommander(){
  if (gridMapSub){
    delete gridMapSub;
    gridMapSub = NULL;
  }  
  if (tfgridMapSub){
    delete tfgridMapSub;
    tfgridMapSub = NULL;
  }
  // if (land_points){
  //   delete land_points;
  //   land_points = NULL;
  // }
}


void LandingCommander::mainCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridMap, const mavros_msgs::State::ConstPtr& state, const mavros_msgs::ExtendedState::ConstPtr& extendedState){
  tf::StampedTransform worldToSensorTf;
  tfListener.lookupTransform(gridMap->header.frame_id, baseFrameId, gridMap->header.stamp, worldToSensorTf);
  tf::Point robotOriginTf = worldToSensorTf.getOrigin();
  robotPose(0) = robotOriginTf.x(); robotPose(1) = robotOriginTf.y();
  
  toMatrix(*gridMap, OccupancyGridEigen);

  landState = extendedState->landed_state;
  armed = state->armed;
  mode = state->mode;

  double res = gridMap->info.resolution;
  int subGridRadius = metersToGrids(safetyRadius, res);

  if (safetyArea){
    checkEmMarkEm(OccupancyGridEigen,subGridRadius);
  }
  splincheckStride(OccupancyGridEigen, land_points, stride, robotPose, subGridRadius);
  // splitncheckExpirimental(OccupancyGridEigen, robotPose, res, safetyRadius*2, land_points, land2base);

  if (debug){ 
    ROS_WARN("Robot Position X: %i, Y: %i", robotPose(0), robotPose(1));
    Debug(OccupancyGridEigen, land_points);
  }

  if (publishOccupancy){
    toOccupancyGrid(OccupancyGridEigen, gridMapOutput, gridMap->info, gridMap->header);
    occupancyPub.publish(gridMapOutput);
  }
  haveOccupancyGridEigen = true;
}


void LandingCommander::splincheckStride(const Eigen::MatrixXi& matrix, Eigen::MatrixX4i& land_waypoints, const int& stride_, const Eigen::Array2i& robotIndex, const int& radius){
  auto start = std::chrono::high_resolution_clock::now();
  int solutions_sum = 0;
  int width = matrix.rows();
  int height = matrix.cols();
  if (width>radius*2+stride_ && height>radius*2+stride_){
    for (int i=0+radius; i<matrix.rows()-radius; i=i+stride_){
      for (int j=0+radius; j<matrix.cols()-radius; j=j+stride_){
        Eigen::MatrixXi submap;
        submap = matrix.block(i-radius,j-radius,radius*2+1,radius*2+1);
        bool occupied = checkArea(submap);
        if (!occupied){
          Eigen::Matrix<int,1,4> solution;
          solution(0) = i;
          solution(1) = j;
          int dist2robot = std::sqrt(std::pow((robotIndex(0)-i), 2)+std::pow((robotIndex(1)-j), 2));
          int dist2base = std::sqrt(std::pow(-i, 2)+std::pow(-j,2));
          solution(2) = dist2robot;
          solution(3) = dist2base;
          land_waypoints.conservativeResize(solutions_sum+1,4);
          land_waypoints.block(solutions_sum,0,1,4) = solution;
          solutions_sum++;

        }
      }
    }
  }else{
    Eigen::Matrix<int,1,4> solution;
    solution(0) = 0;
    solution(1) = 0;
    int dist2robot = std::sqrt(std::pow((robotIndex(0)), 2)+std::pow((robotIndex(1)), 2));
    int dist2base = 0;
    solution(2) = dist2robot;
    solution(3) = dist2base;
    land_waypoints.resize(1,4);
    land_waypoints.block(solutions_sum,0,1,4) = solution;
    land2base = true;
  }
  if (land2base){
    land_waypoints = sortDescOrder(land_waypoints, 3);
  }else{
    land_waypoints = sortDescOrder(land_waypoints,2);
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  if (debug){ROS_WARN("splincheckStride function executred in %i", duration);}
}


bool LandingCommander::toMatrix(const nav_msgs::OccupancyGrid& occupancyGrid, Eigen::MatrixXi& matrix){
  auto start = std::chrono::high_resolution_clock::now();
  int cols_ = occupancyGrid.info.height;
  assert(cols_>0);
  int rows_ = occupancyGrid.info.width;
  assert(rows_>0);
  matrix.resize(rows_,cols_);
  //Occupancy grid is ROW MAJOR but we will treat it as collumn major for our sanity's sake
  for (int i=0;i<occupancyGrid.data.size();i++) {
    Eigen::Array2i IndexXY; 
    IndexXY = getIndexFromLinearIndex(rows_, i);
    matrix(IndexXY(0),IndexXY(1))=occupancyGrid.data[i];
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  if (debug){ROS_WARN("toMatrix function executred in %i", duration);}
  return true;
}


bool LandingCommander::toOccupancyGrid(const Eigen::MatrixXi& matrix, nav_msgs::OccupancyGrid& occupancyGrid, const nav_msgs::MapMetaData& mapMetaData, const std_msgs::Header& header){
  auto start = std::chrono::high_resolution_clock::now();
  occupancyGrid.data.clear();
  occupancyGrid.info = mapMetaData;
  occupancyGrid.header = header;
  int rows_ = matrix.rows();
  int cols_ = matrix.cols();
  occupancyGrid.info.width = rows_;
  occupancyGrid.info.height = cols_;
  for (int i=0; i<cols_; i++){
    for (int j=0; j<rows_; j++){
      occupancyGrid.data.push_back(matrix(j,i));
    }
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  if (debug){ROS_WARN("toOccupancyGrid function executed in %i", duration);}
  return true;
}

//rewrite with eigen block operations
void LandingCommander::checkEmMarkEm(Eigen::MatrixXi& matrix, const int& radius){
  auto start = std::chrono::high_resolution_clock::now();
  Eigen::MatrixXi newMatrix(matrix.rows()+radius*2, matrix.cols()+radius*2);
  newMatrix.block(radius,radius,matrix.rows(),matrix.cols()) = matrix;
  for (int i=0;i<matrix.rows();i++){
    for (int j=0;j<matrix.cols();j++){
      if (matrix(i,j)==100){
        Eigen::MatrixXi submap(radius*2+1,radius*2+1);
        submap.setConstant(100);
        newMatrix.block(i-radius,j-radius,submap.rows(),submap.cols()) = submap;
      }
    }
  }
  matrix = newMatrix.block(radius,radius,matrix.cols(),matrix.rows());
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  if (debug){ROS_WARN("checkEmMarkEm function executred in %i", duration);}
}


void LandingCommander::Debug( Eigen::MatrixXi& matrix, const Eigen::MatrixX4i& land_waypoints){
  if (land_waypoints.rows()>0){
    for (int i=0;i<land_waypoints.rows();i++){
      int x = land_waypoints(i,0);
      int y = land_waypoints(i,1);
      matrix(x, y)=-1;
      int dist2robot = land_waypoints(i,2);
      int dist2base = land_waypoints(i,3);
      // ROS_WARN("Land Waypoint (%i, %i), dist2robot: %i, dist2base: %i", x,y,dist2robot,dist2base);
    }
  }
}


void LandingCommander::handleWaypoint(const ros::TimerEvent&){
  bool land_cmd_sent = false;
  bool pos_cmd_sent = false;
  bool have_land_point = false;
  Eigen::Array2i land_target;
  land_target.setZero();
  geometry_msgs::PoseStamped land_pose;

  if (haveOccupancyGridEigen){
    if (land_points.rows()>0 && !have_land_point)
    {
      land_target(0)=land_points(0,0);
      land_target(1)=land_points(0,1);

      land_pose.pose.position.x = land_target(0);
      land_pose.pose.position.y = land_target(0);
      land_pose.pose.position.z = 10;
      
      for(int i = 20; ros::ok() && i > 0; --i){
        pos_setpoint.publish(land_pose);
      }
      have_land_point = true;
    }

    if ((OccupancyGridEigen(land_target(0),land_target(1))!=100) && have_land_point){
      pos_setpoint.publish(land_pose);

      mavros_msgs::SetMode land_set_mode;
      mavros_msgs::SetMode position_set_mode;

      land_set_mode.request.custom_mode = "AUTO.LAND";
      position_set_mode.request.custom_mode = "POSCTL";

      if (waypointReached(robotPose, land_target) && mode=="OFFBOARD" && landState!=1){
        if(set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
          ROS_INFO("Land mode enabled");
        }
      }

      if (landState==1 && armed && mode!="POSCTL"){
        if(set_mode_client.call(position_set_mode) && position_set_mode.response.mode_sent){
          ROS_INFO("Switched back to position mode");
          have_land_point = false;
        }
      }
    }

  }
}


}