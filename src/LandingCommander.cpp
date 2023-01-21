#include <landing_commander/LandingCommander.h>

namespace landing_commander{

LandingCommander::LandingCommander(const ros::NodeHandle &nh_)
: nodeHandle(nh_),
  gridMapSub(NULL),
  tfgridMapSub(NULL),
  fcuStateSub(NULL),
  fcuExtendedStateSub(NULL),
  sync(NULL),
  land_pose_dist(0),
  enableGuard(false),
  land_point_serching(true),
  baseFrameId("base_link"),
  safetyRadius(2.0),
  ratio(0.0),
  stride_coef(0.055),
  targetProcTime(50.0),
  OccupancyGridEigen(0,0),
  latchedTopics(true),
  minStride(1.0),
  land2base(false),
  land_points(0,3),
  active_land_point(1,3),
  safetyArea(true),
  debug(true),
  publishOccupancy(true),
  land_points_temp(0,3)
  {
    nodeHandle.param("safety_dist", safetyRadius, safetyRadius);
    nodeHandle.param("target_proc_time", targetProcTime, targetProcTime);
    nodeHandle.param("safety_area", safetyArea, safetyArea);
    nodeHandle.param("minimum_strde", minStride, minStride);
    nodeHandle.param("stride_coef", stride_coef, stride_coef);
    nodeHandle.param("robot_frame", baseFrameId, baseFrameId);
    nodeHandle.param("land_to_base", land2base, land2base);
    nodeHandle.param("debug",debug,debug);
    nodeHandle.param("show_occupancy_map", publishOccupancy,publishOccupancy);
    nodeHandle.param("mark_safety_area",safetyArea, safetyArea);

    occupancyPub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("occupancy_output", 5, latchedTopics);

    if (debug) {
      debugger = nodeHandle.advertise<landing_commander::LandingCommanderDebug>("Landing_Commander_Debug", 5, latchedTopics);
      debugger_landingPoints = nodeHandle.advertise<landing_commander::LandingTargets>("Debug_Landing_Targets", 5, latchedTopics);
      }

    pos_setpoint = nodeHandle.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    commanderTimer = nodeHandle.createTimer(ros::Rate(2), &LandingCommander::commander, this);

    set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");



    gridMapSub = new message_filters::Subscriber<nav_msgs::OccupancyGrid>(nodeHandle, "/projected_map", 10);
    tfgridMapSub = new tf::MessageFilter<nav_msgs::OccupancyGrid>(*gridMapSub, tfListener, baseFrameId, 10);
    fcuStateSub = new message_filters::Subscriber<mavros_msgs::State>(nodeHandle, "/mavros/state", 10);
    fcuExtendedStateSub = new message_filters::Subscriber<mavros_msgs::ExtendedState>(nodeHandle, "/mavros/extended_state", 10);

    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *tfgridMapSub, *fcuStateSub, *fcuExtendedStateSub);

    sync->registerCallback(boost::bind(&LandingCommander::mainCallback, this, boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3));

    land_points.conservativeResize(1,3);
    land_points.setZero();

    land_set_mode.request.custom_mode = "AUTO.LAND";
    position_set_mode.request.custom_mode = "POSCTL";
    offboard_set_mode.request.custom_mode = "OFFBOARD";

    active_land_point(0) = rand()% 21;
    active_land_point(1) = rand()% 21;
    active_land_point(2) = rand()% 21;
  }

LandingCommander::~LandingCommander(){
  if (gridMapSub){
    delete gridMapSub;
    gridMapSub = NULL;
  }  
  if (sync){
    delete sync;
    sync = NULL;
  }
  
  if (tfgridMapSub){
    delete tfgridMapSub;
    tfgridMapSub = NULL;
  }
}


void LandingCommander::mainCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridMap, const mavros_msgs::State::ConstPtr& state, const mavros_msgs::ExtendedState::ConstPtr& extendedState){
  auto start = std::chrono::high_resolution_clock::now();
  tf::StampedTransform worldToSensorTf;
  tfListener.lookupTransform(gridMap->header.frame_id, baseFrameId, gridMap->header.stamp, worldToSensorTf);
  tf::Point robotOriginTf = worldToSensorTf.getOrigin();
  robotPose(0) = robotOriginTf.x(); robotPose(1) = robotOriginTf.y();     
  
  toMatrix(*gridMap, OccupancyGridEigen, ratio);
  if (debug){debug_msg.ratio = ratio;}

  //Info to be passed to commander
  landState = extendedState->landed_state;
  armed = state->armed;
  mode = state->mode;
  map_origin = gridMap->info.origin;

  double res = gridMap->info.resolution;
  int subGridRadius = metersToGrids(safetyRadius, res);

  if (safetyArea){
    checkEmMarkEm(OccupancyGridEigen,subGridRadius);
  }

  splincheckStride(OccupancyGridEigen, gridMap->info.origin, land_points, ratio, stride_coef, targetProcTime, robotPose, subGridRadius);
  //land_points are in occupancy's grid coords from now on (with proper origin)
  if (debug){ 
    Debug(OccupancyGridEigen, land_points, gridMap->info.origin);
  }

  if (publishOccupancy){
    toOccupancyGrid(OccupancyGridEigen, gridMapOutput, gridMap->info, gridMap->header);
    occupancyPub.publish(gridMapOutput);
  }
  haveOccupancyGridEigen = true;

  //Populate Debug messages
  if(debug){
    debug_msg.robot_position.x = robotPose(0);
    debug_msg.robot_position.y = robotPose(1);

    debug_msg.landing_target.x = land_points(0,0);
    debug_msg.landing_target.y = land_points(0,1);
    debug_msg.landing_target.distance = land_points(0,2);

    debug_msg.active_land_target.x = active_land_point(0);
    debug_msg.active_land_target.y = active_land_point(1);
    debug_msg.active_land_target.distance = active_land_point(2);

    debug_msg.land_point_serching = land_point_serching;
    debug_msg.land_points_sum = land_points.rows();

    debug_msg.Header.stamp = ros::Time::now();
    debugger.publish(debug_msg);

    landingTargets_msg.Header.stamp = ros::Time::now();
    landingTargets_msg.landing_targets.clear();
    for (int i=0; i<land_points.rows(); i++){
      landingTarget_msg.x = land_points(i,0);
      landingTarget_msg.y = land_points(i,1);
      landingTarget_msg.distance = land_points(i,2);
      landingTargets_msg.landing_targets.push_back(landingTarget_msg);
    }
    debug_msg.land_points_sum = land_points.rows();
    debugger_landingPoints.publish(landingTargets_msg);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
    debug_msg.total_time = duration.count()*0.000001;
  }
}


void LandingCommander::splincheckStride(
  const Eigen::MatrixXi& matrix,
  const geometry_msgs::Pose& origin, 
  Eigen::MatrixX3i& land_waypoints, 
  const double& ratio, 
  const double& stride_coef_, 
  const double& targetProcTime_, 
  const Eigen::Array2i& robotIndex, 
  const int& radius)
  {
  auto start = std::chrono::high_resolution_clock::now();
  double strideD = (std::sqrt(((std::pow(((2*safetyRadius)+1),2)*matrix.rows()*matrix.cols())-(matrix.rows()*matrix.cols()*ratio))/targetProcTime_))*stride_coef_;
  if (strideD<minStride){strideD=minStride;}
  int stride = std::round(strideD);
  int solutions_sum = 0;
  int width = matrix.rows();
  int height = matrix.cols();
  for (int i=0+radius; i<matrix.rows()-radius; i=i+stride){
    for (int j=0+radius; j<matrix.cols()-radius; j=j+stride){
      Eigen::MatrixXi submap;
      submap.resize(radius*2+1,radius*2+1);
      submap = matrix.block(i-radius,j-radius,radius*2+1,radius*2+1);
      bool occupied = checkArea(submap);
      if (!occupied){
        Eigen::Matrix<int,1,3> solution;
        solution(0) = i;
        solution(1) = j;
        solution = fromMatrixToRealWorld(solution, origin);
        if (land2base){
          Eigen::Matrix<int,1,2> base;
          base.setZero();
          solution(2) = euclidianDistance(solution.block(0,0,1,2), base);
        }else{
          solution(2) = euclidianDistance(solution.block(0,0,1,2), robotIndex);
        }
        land_waypoints.conservativeResize(solutions_sum+1,3);
        land_waypoints.block(solutions_sum,0,1,3) = solution;
        solutions_sum++;
      }
    }
  }

  land_waypoints = sortAscOrder(land_waypoints);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
  if (debug){
    debug_msg.splitNCheckStride_time = duration.count()*0.000001;
    debug_msg.stride = stride;
    }
}

void LandingCommander::toMatrix(const nav_msgs::OccupancyGrid& occupancyGrid, Eigen::MatrixXi& matrix, double& ratio_){
  auto start = std::chrono::high_resolution_clock::now();
  int cols_ = occupancyGrid.info.height;
  assert(cols_>0);
  int rows_ = occupancyGrid.info.width;
  assert(rows_>0);
  matrix.resize(rows_,cols_);
  int occupied_counter=0;
  //Occupancy grid is ROW MAJOR but we will treat it as collumn major for our sanity's sake
  for (int i=0;i<occupancyGrid.data.size();i++) {
    Eigen::Array2i IndexXY; 
    IndexXY = getIndexFromLinearIndex(rows_, i);
    matrix(IndexXY(0),IndexXY(1))=occupancyGrid.data[i];
    if (matrix(IndexXY(0),IndexXY(1))==100){occupied_counter++;}
  }
  int matrix_size = matrix.rows()*matrix.cols();
  ratio_ = (double)occupied_counter/matrix_size;
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
  if (debug){debug_msg.toMatrix_time = duration.count()*0.000001;}
}


void LandingCommander::toOccupancyGrid(const Eigen::MatrixXi& matrix, nav_msgs::OccupancyGrid& occupancyGrid, const nav_msgs::MapMetaData& mapMetaData, const std_msgs::Header& header){
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
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
  if (debug){debug_msg.toOccupancyGrid_time = duration.count()*0.000001;}
}

void LandingCommander::checkEmMarkEm(Eigen::MatrixXi& matrix, const int& radius){
  auto start = std::chrono::high_resolution_clock::now();
  Eigen::MatrixXi newMatrix;
  newMatrix.resize(matrix.rows()+radius*2, matrix.cols()+radius*2);
  newMatrix.setConstant(0);
  newMatrix.block(radius,radius,matrix.rows(),matrix.cols()) = matrix;
  for (int i=0;i<matrix.rows();i++){
    for (int j=0;j<matrix.cols();j++){
      if (matrix(i,j)==100){
        newMatrix.block(i,j,radius*2+1,radius*2+1).setConstant(100);
      }
    }
  }
  matrix = newMatrix.block(radius,radius,matrix.rows(),matrix.cols());
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
  if (debug){debug_msg.checkEmMarkEm_time = duration.count()*0.000001;}
}


void LandingCommander::Debug( Eigen::MatrixXi& matrix, const Eigen::MatrixX3i& land_waypoints, const geometry_msgs::Pose& origin){
  if (land_waypoints.rows()>0){
    for (int i=0;i<land_waypoints.rows();i++){
      Eigen::Matrix<int,1,3> land_waypoint_matrix(land_waypoints(i,0),land_waypoints(i,1),land_waypoints(i,2));
      land_waypoint_matrix = fromRealWorldToMatrix(land_waypoint_matrix, origin);
      int x = land_waypoint_matrix(0);
      int y = land_waypoint_matrix(1);
      if (i==0){
        matrix(x,y)=-150;
      }else{
        matrix(x,y)=-120;
      }
    }
  }
}


void LandingCommander::checkLandingPoint(){
  if ( !validPoint(active_land_point, land_points_temp) && land_point_serching==false ){
    land_point_serching = true;
    if (active_land_point.isZero() && land2base){
      land2base = false;
    }
  }
}

void LandingCommander::fallbackSafety(){
  if (landState=2 && !validPoint(active_land_point, land_points_temp)){
    if(set_mode_client.call(offboard_set_mode) && position_set_mode.response.mode_sent){
      if (debug){ROS_INFO("Switched back to offboard mode");}
    }  
  }
}


void LandingCommander::commander(const ros::TimerEvent&){
  if (haveOccupancyGridEigen){
    land_points_temp.resize(land_points.rows(), land_points.cols());
    land_points_temp = land_points;
    if (mode=="OFFBOARD"){
      land_point_serching = false;
    }else if (mode=="POSCTL"){
      land_point_serching = true;
    }

    //check if landing point is occupied;
    checkLandingPoint();

    if(land_point_serching){
      active_land_point = land_points_temp.block(0,0,1,3);
      land_pose.pose.position.x = land_points_temp(0,0);
      land_pose.pose.position.y = land_points_temp(0,1);
      land_pose.pose.position.z = 10;
      land_pose_dist = land_points_temp(0,2);
      land_point_serching = false;
      for(int i = 50; ros::ok() && i > 0; --i){
        pos_setpoint.publish(land_pose);
      }
    }

    if (!land_point_serching){
      pos_setpoint.publish(land_pose);

      if (waypointReached(robotPose, land_pose.pose.position) && mode=="OFFBOARD" && landState!=1){
        if(set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
          if(debug){ROS_INFO("Land mode enabled");}
        }
      }
      fallbackSafety();
      if (landState==1 && armed && mode!="POSCTL"){
        if(set_mode_client.call(position_set_mode) && position_set_mode.response.mode_sent){
          if (debug){ROS_INFO("Switched back to position mode");}
        }
      }
    }
  }
}
}