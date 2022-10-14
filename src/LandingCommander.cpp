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
  have_land_point(false),
  baseFrameId("base_link"),
  safetyRadius(2.0),
  ratio(0.0),
  coefficients(0.0717,142.88,-80.628),
  targetProcTime(200.0),
  OccupancyGridEigen(0,0),
  latchedTopics(true),
  land2base(false),
  land_points(0,3),
  safetyArea(true),
  debug(true),
  publishOccupancy(true),
  haveOccupancyGridEigen(false)
  {
    nodeHandle.param("safety_dist", safetyRadius, safetyRadius);
    nodeHandle.param("robot_frame", baseFrameId, baseFrameId);
    nodeHandle.param("land_to_base", land2base, land2base);
    nodeHandle.param("debug",debug,debug);
    nodeHandle.param("show_occupancy_map", publishOccupancy,publishOccupancy);
    nodeHandle.param("mark_safety_area",safetyArea, safetyArea);

    occupancyPub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("occupancy_output", 5, latchedTopics);
    pos_setpoint = nodeHandle.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    commanderTimer = nodeHandle.createTimer(ros::Rate(20), &LandingCommander::commander, this);
    guardTimer = nodeHandle.createTimer(ros::Rate(2), &LandingCommander::guard, this);

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
}


void LandingCommander::mainCallback(const nav_msgs::OccupancyGrid::ConstPtr& gridMap, const mavros_msgs::State::ConstPtr& state, const mavros_msgs::ExtendedState::ConstPtr& extendedState){
  tf::StampedTransform worldToSensorTf;
  tfListener.lookupTransform(gridMap->header.frame_id, baseFrameId, gridMap->header.stamp, worldToSensorTf);
  tf::Point robotOriginTf = worldToSensorTf.getOrigin();
  robotPose(0) = robotOriginTf.x(); robotPose(1) = robotOriginTf.y();     
  
  toMatrix(*gridMap, OccupancyGridEigen, ratio);
  if (debug){ROS_WARN("Ratio number = %f", ratio);}

  landState = extendedState->landed_state;
  armed = state->armed;
  mode = state->mode;

  double res = gridMap->info.resolution;
  int subGridRadius = metersToGrids(safetyRadius, res);

  if (safetyArea){
    checkEmMarkEm(OccupancyGridEigen,subGridRadius);
  }
  splincheckStride(OccupancyGridEigen, gridMap->info.origin, land_points, ratio, coefficients, targetProcTime, robotPose, subGridRadius);
  //land_points are in occupancy's grid coords from now on (with proper origin)
  if (debug){ 
    ROS_WARN("Robot Position X: %i, Y: %i", robotPose(0), robotPose(1));
    Debug(OccupancyGridEigen, land_points, gridMap->info.origin);
  }

  if (publishOccupancy){
    toOccupancyGrid(OccupancyGridEigen, gridMapOutput, gridMap->info, gridMap->header);
    occupancyPub.publish(gridMapOutput);
  }
  haveOccupancyGridEigen = true;
}


void LandingCommander::splincheckStride(
  const Eigen::MatrixXi& matrix,
  const geometry_msgs::Pose& origin, 
  Eigen::MatrixX3i& land_waypoints, 
  const double& ratio, 
  const Eigen::Matrix<double,1,3>& coefficients_, 
  const double& targetProcTime_, 
  const Eigen::Array2i& robotIndex, 
  const int& radius)
  {
  double strideD = -((coefficients_(0)*(matrix.rows()*matrix.cols())+coefficients_(1)*ratio-targetProcTime_)/coefficients_(2));
  if (strideD<1.0){strideD=1.0;}
  int stride = std::round(strideD);
  auto start = std::chrono::high_resolution_clock::now();
  int solutions_sum = 0;
  int width = matrix.rows();
  int height = matrix.cols();
  for (int i=0+radius; i<matrix.rows()-radius; i=i+stride){
    for (int j=0+radius; j<matrix.cols()-radius; j=j+stride){
      Eigen::MatrixXi submap;
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
          solution(2) = euclidianDistance(solution.block(0,0,1,2), robotIndex);;
        }
        land_waypoints.conservativeResize(solutions_sum+1,3);
        land_waypoints.block(solutions_sum,0,1,3) = solution;
        solutions_sum++;

      }
    }
  }
  if(land_waypoints.rows()==0){
    land_waypoints.conservativeResize(1,3);
    land_waypoints.setZero();
  }

  land_waypoints = sortAscOrder(land_waypoints);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  if (debug){ROS_WARN("splincheckStride function executred in %i with stride %i", duration, stride);}
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
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  if (debug){ROS_WARN("toMatrix function executred in %i", duration);}
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
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  if (debug){ROS_WARN("toOccupancyGrid function executed in %i", duration);}
}

void LandingCommander::checkEmMarkEm(Eigen::MatrixXi& matrix, const int& radius){
  auto start = std::chrono::high_resolution_clock::now();
  Eigen::MatrixXi newMatrix(matrix.rows()+radius*2, matrix.cols()+radius*2);
  newMatrix.setConstant(0);
  newMatrix.block(radius,radius,matrix.rows(),matrix.cols()) = matrix;
  Eigen::MatrixXi submap(radius*2+1,radius*2+1);
  submap.setConstant(100);  
  for (int i=0;i<matrix.rows();i++){
    for (int j=0;j<matrix.cols();j++){
      if (matrix(i,j)==100){
        newMatrix.block(i,j,submap.rows(),submap.cols()) = submap;
      }
    }
  }
  matrix = newMatrix.block(radius,radius,matrix.rows(),matrix.cols());
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  if (debug){ROS_WARN("checkEmMarkEm function executred in %i", duration);}
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
        ROS_WARN("nearest land_waypoint %i,%i, distance to taget %i", land_waypoints(0,0), land_waypoints(0,1), land_waypoints(0,2));
      }else{
        matrix(x,y)=-10;
      }
      // ROS_WARN("Waypoints (%i,%i), distance: %i", land_waypoints(i,0),land_waypoints(i,1),land_waypoints(i,2));
    }
  }
}


void LandingCommander::commander(const ros::TimerEvent&){

  if (haveOccupancyGridEigen){

    if (!have_land_point){
      land_pose.pose.position.x = land_points(0,0);
      land_pose.pose.position.y = land_points(0,1);
      land_pose.pose.position.z = 10;
      land_pose_dist = land_points(0,2);
      for(int i = 50; ros::ok() && i > 0; --i){
        pos_setpoint.publish(land_pose);
      }
    }

    if (mode=="OFFBOARD"){
      have_land_point=true;
    }else{
      have_land_point=false;
    }

    if (have_land_point){
      pos_setpoint.publish(land_pose);

      land_set_mode.request.custom_mode = "AUTO.LAND";
      position_set_mode.request.custom_mode = "POSCTL";

      if (waypointReached(robotPose, land_pose.pose.position) && mode=="OFFBOARD" && landState!=1){
        if(set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
          if(debug){ROS_INFO("Land mode enabled");}
        }
      }

      if (landState==1 && armed && mode!="POSCTL"){
        if(set_mode_client.call(position_set_mode) && position_set_mode.response.mode_sent){
          if (debug){ROS_INFO("Switched back to position mode");}
        }
      }
    }
  }
}

void LandingCommander::guard(const ros::TimerEvent&){
  if (mode=="AUTO.LAND" && OccupancyGridEigen(robotPose(0),robotPose(1))==100){
    have_land_point=false;
  }
}

}