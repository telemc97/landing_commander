#include <landing_commander/LandingCommander.h>

namespace landing_commander{

LandingCommander::LandingCommander(const ros::NodeHandle &nh_)
: nodeHandle(nh_),
  gridMapSub(NULL),
  tfgridMapSub(NULL),
  land_points(NULL),
  baseFrameId("base_link"),
  safetyRadius(2.0),
  latchedTopics(true),
  minimumLandingArea(3.0),
  land2base(false),
  debug(false),
  publishOccupancy(true)
  {
    nodeHandle.param("safety_dist", safetyRadius, safetyRadius);
    nodeHandle.param("robot_frame", baseFrameId, baseFrameId);
    nodeHandle.param("minimum_landing_area", minimumLandingArea, minimumLandingArea);
    nodeHandle.param("land_to_base", land2base, land2base);
    nodeHandle.param("debug",debug,debug);
    nodeHandle.param("show_occupancy_map", publishOccupancy,publishOccupancy);

    occupancyPub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("occupancy_output", 5, latchedTopics);
    pos_setpoint = nodeHandle.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    set_mode_client = nodeHandle.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    gridMapSub = new message_filters::Subscriber<nav_msgs::OccupancyGrid>(nodeHandle, "/projected_map", 10);
    tfgridMapSub = new tf::MessageFilter<nav_msgs::OccupancyGrid>(*gridMapSub, tfListener, baseFrameId, 10);
    tfgridMapSub->registerCallback(boost::bind(&LandingCommander::gridHandler, this, boost::placeholders::_1));

    land_points = new Eigen::MatrixX4i(0,4);
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
  
  if (land_points){
    delete land_points;
    land_points = NULL;
  }
}


void LandingCommander::gridHandler(const nav_msgs::OccupancyGrid::ConstPtr& gridMap){

  tf::StampedTransform worldToSensorTf;
  tfListener.lookupTransform(gridMap->header.frame_id, baseFrameId, gridMap->header.stamp, worldToSensorTf);
  tf::Point robotOriginTf = worldToSensorTf.getOrigin();
  Eigen::Array2i robotPose;
  robotPose(0) = robotOriginTf.x();
  robotPose(1) = robotOriginTf.y();

  toMatrix(*gridMap, OccupancyGridEigen);
  double res = gridMap->info.resolution;
  int subGridRadius = checkNeighborsRadius(safetyRadius, res);
  OccupancyGridEigen = checkEmMarkEm(OccupancyGridEigen,subGridRadius);

  LandingCommander::splitncheck(OccupancyGridEigen, robotPose, res, minimumLandingArea, *land_points, land2base);

  if (debug){ 
    ROS_WARN("Robot Position X: %i, Y: %i", robotPose(0), robotPose(1));
    OccupancyGridEigen = Debug(OccupancyGridEigen, *land_points);
  }

  if (publishOccupancy){
    toOccupancyGrid(OccupancyGridEigen, gridMapOutput, gridMap->info, gridMap->header);
    occupancyPub.publish(gridMapOutput);
  }

}


bool LandingCommander::splitncheck(const Eigen::MatrixXi& matrix, Eigen::Array2i& robotIndex, double resolution, double minLandA, Eigen::MatrixX4i& land_waypoints, bool& land2base, int offset_w, int offset_h){
  int rows = matrix.rows();
  int cols = matrix.cols();
  int w_div = LandingCommander::maxDivider(rows);
  int h_div = LandingCommander::maxDivider(cols);
  if ((w_div*resolution<minLandA) & (h_div*resolution<minimumLandingArea)){
    int submaps = w_div*h_div;
    Eigen::MatrixXi score(w_div,h_div);
    int sub_w = rows/w_div;
    int sub_h = cols/h_div;
    for (int i=0; i<w_div; i++){
      for (int j=0; j<h_div; j++){
        int occ_cells = 0;
        for (int it_rows=sub_w*i+offset_w; it_rows<sub_w+sub_w*i+offset_w; it_rows++){
          for (int it_cols=sub_h*j+offset_h; it_cols<sub_h+sub_h*j+offset_h; it_cols++){
            if (matrix(it_rows, it_cols)==100){
              occ_cells++;
            }
          }
        }
        score(i,j) = occ_cells;
      }
    }
    int best = score(0,0);
    int solutions = 0;
    for (int i=0; i<w_div;i++){
      for (int j=0;j<h_div;j++){
        if (score(i,j)<best){
          best = score(i,j);
        }
        if (score(i,j)==0){solutions++;}
      }
    }
    if (best==0){
      land_waypoints.resize(solutions,4);
      int i=0;
      for (int j=0;j<w_div;j++){
        for (int k=0;k<h_div;k++){
          if (score(j,k)==0){
            int centroid_w = (sub_w/2)+(sub_w*j)+offset_w;
            int centroid_h = (sub_h/2)+(sub_h*k)+offset_h;
            int dist2robot = std::sqrt(std::pow((robotIndex(0)-centroid_w), 2)+std::pow((robotIndex(1)-centroid_h), 2));
            int dist2base = std::sqrt(std::pow(-centroid_w, 2)+std::pow(-centroid_h,2));
            land_waypoints(i,0) = centroid_w;
            land_waypoints(i,1) = centroid_h;
            land_waypoints(i,2) = dist2robot;
            land_waypoints(i,3) = dist2base;
            i++;
          }
        }
      }
      Eigen::Matrix<int,1,4> temp;
      if (land2base){
        for (int i=0;i<solutions;i++){
          for (int j=i+1;j<solutions;j++){
            if (land_waypoints(i,2)>land_waypoints(j,2)){
              temp = land_waypoints.block(i,0,1,4);
              land_waypoints.block(i,0,1,4) = land_waypoints.block(j,0,1,4);
              land_waypoints.block(j,0,1,4) = temp;
            }
          }
        }
      }else{
        for (int i=0;i<solutions;i++){
          for (int j=i+1;j<solutions;j++){
            if (land_waypoints(i,3)>land_waypoints(j,3)){
              temp = land_waypoints.block(i,0,1,4);
              land_waypoints.block(i,0,1,4) = land_waypoints.block(j,0,1,4);
              land_waypoints.block(j,0,1,4) = temp;
            }
          }
        }
      }
    }else{
      for(int i=0;i<w_div;i++){
        for(int j=0;j<h_div;j++){
          Eigen::MatrixXi new_submap = matrix.block((sub_w*i)+offset_w, (sub_h*j)+offset_h, sub_w, sub_h);
          int new_offset_w = (sub_w*i)+offset_w;
          int new_offset_h = (sub_h*j)+offset_h;
          LandingCommander::splitncheck(new_submap, robotIndex, resolution, minLandA, land_waypoints, land2base, new_offset_w, new_offset_h);
        }
      }
    }
  }
  return true;
}


bool LandingCommander::toMatrix(const nav_msgs::OccupancyGrid& occupancyGrid, Eigen::MatrixXi& gridEigen){
  int cols_ = occupancyGrid.info.height;
  int rows_ = occupancyGrid.info.width;
  gridEigen.resize(rows_,cols_);
  //Occupancy grid is ROW MAJOR but we will treat it as collumn major for our sanity's sake
  for (int i=0;i<occupancyGrid.data.size();i++) {
    Eigen::Array2i IndexXY; 
    IndexXY = getIndexFromLinearIndex(rows_, i);
    gridEigen(IndexXY(0),IndexXY(1))=occupancyGrid.data[i];
  }
  // gridEigen(0,1)=100;
  return true;
}


bool LandingCommander::toOccupancyGrid(const Eigen::MatrixXi& gridEigen, nav_msgs::OccupancyGrid& occupancyGrid, const nav_msgs::MapMetaData& mapMetaData, const std_msgs::Header& header){
  occupancyGrid.data.clear();
  occupancyGrid.info = mapMetaData;
  occupancyGrid.header = header;
  int rows_ = gridEigen.rows();
  int cols_ = gridEigen.cols();
  occupancyGrid.info.width = rows_;
  occupancyGrid.info.height = cols_;
  for (int i=0; i<cols_; i++){
    for (int j=0; j<rows_; j++){
      occupancyGrid.data.push_back(gridEigen(j,i));
    }
  }
  return true;
}


int LandingCommander::checkNeighborsRadius (const double& safeAreaRadius, const double& resolution){
  int gridNumRadius;
  gridNumRadius = safeAreaRadius/resolution;
  return gridNumRadius;
}


Eigen::MatrixXi LandingCommander::checkEmMarkEm(const Eigen::MatrixXi& matrix, int& radius){
  Eigen::MatrixXi newMatrix;
  newMatrix.resize(matrix.rows(),matrix.cols());
  for (int i=0;i<matrix.rows();i++){
    for (int j=0;j<matrix.cols();j++){
      if (matrix(i,j)==100){
        int subIndex_x, subIndex_y;
        int maxSubIndex_x,maxSubIndex_y;
        if (i-radius<0){subIndex_x=0;}
        else{subIndex_x = i-radius;}
        if (j-radius<0){subIndex_y=0;}
        else{subIndex_y = j-radius;}
        if (i+radius>matrix.rows()){maxSubIndex_x=matrix.rows()-1;}
        else{maxSubIndex_x = i+radius;}
        if (j+radius>matrix.cols()){maxSubIndex_y=matrix.cols()-1;}
        else{maxSubIndex_y = j+radius;}
        for (int k=subIndex_x;k<maxSubIndex_x;k++){
          for (int l=subIndex_y;l<maxSubIndex_y;l++){
            newMatrix(k,l)=100;
          }
        }
      }else{
        newMatrix(i,j)=matrix(i,j);
      }
    }
  }
  return newMatrix;
}


Eigen::MatrixXi LandingCommander::Debug( Eigen::MatrixXi& matrix, Eigen::MatrixX4i& land_waypoints){
  for (int i=0;i<land_waypoints.rows();i++){
    matrix(land_waypoints(i,0),land_waypoints(i,1))=-1;
    int x = land_waypoints(i,0);
    int y = land_waypoints(i,1);
    int dist2robot = land_waypoints(i,2);
    int dist2base = land_waypoints(i,3);
    ROS_WARN("Land Waypoint (%i, %i), dist2robot: %i, dist2base: %i", x,y,dist2robot,dist2base);
  }
  return matrix;
}

void LandingCommander::handleLand(const Eigen::MatrixX4i& land_waypoints){

  // ros::Rate rate(20.0);
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = land_waypoints(0,0);
  pose.pose.position.y = land_waypoints(0,1);
  pose.pose.position.z = 2;
  
  for(int i = 100; ros::ok() && i > 0; --i){
    pos_setpoint.publish(pose);
    ros::spinOnce();
    // rate.sleep();
  }

  ros::Time last_request = ros::Time::now();

  pos_setpoint.publish(pose);

  // ros::spinOnce();
  // rate.sleep();
}


}