#include <landing_commander/LandingCommander.h>

namespace landing_commander{

LandingCommander::LandingCommander(const ros::NodeHandle &nh_)
: nodeHandle(nh_),
  gridMapSub(NULL),
  tfgridMapSub(NULL),
  baseFrameId("base_link"),
  safetyRadius(2.0),
  latchedTopics(true),
  minimumLandingArea(3.0)
  {
    nodeHandle.param("safety_dist", safetyRadius, safetyRadius);
    nodeHandle.param("robot_frame", baseFrameId, baseFrameId);
    nodeHandle.param("minimum_landing_area", minimumLandingArea, minimumLandingArea);

    mavros_state = nodeHandle.subscribe<mavros_msgs::State>("mavros/state", 10, &LandingCommander::state_cb, this);

    occupancySub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("occupancy_output", 5, latchedTopics);
    pos_setpoint = nodeHandle.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    gridMapSub = new message_filters::Subscriber<nav_msgs::OccupancyGrid>(nodeHandle, "/projected_map", 10);
    tfgridMapSub = new tf::MessageFilter<nav_msgs::OccupancyGrid>(*gridMapSub, tfListener, baseFrameId, 10);
    tfgridMapSub->registerCallback(boost::bind(&LandingCommander::gridHandler, this, boost::placeholders::_1));
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


void LandingCommander::gridHandler(const nav_msgs::OccupancyGrid::ConstPtr& gridMap){
  tf::StampedTransform worldToSensorTf;
  tfListener.lookupTransform(baseFrameId, gridMap->header.frame_id, gridMap->header.stamp, worldToSensorTf);
  tf::Point robotOriginTf = worldToSensorTf.getOrigin();
  int robot_x = robotOriginTf.x();
  int robot_y = robotOriginTf.y();
  toMatrix(*gridMap, OccupancyGridEigen);
  Eigen::Array2i robotPoseIndex;
  robotPoseIndex = (robot_x, robot_y);
  // ROS_WARN("X: %i, Y: %i", robotPoseIndex(0), robotPoseIndex(1));
  double res = gridMap->info.resolution;
  int subGridRadius = checkNeighborsRadius(safetyRadius, res);
  OccupancyGridEigen = checkEmMarkEm(OccupancyGridEigen,subGridRadius);

  Eigen::MatrixX4i land_points;
  LandingCommander::splitncheck(OccupancyGridEigen, robotPoseIndex, res, minimumLandingArea, land_points);

  toOccupancyGrid(OccupancyGridEigen, gridMapOutput, *gridMap);
  occupancySub.publish(gridMapOutput);
}


bool LandingCommander::splitncheck(const Eigen::MatrixXi& mapdata, Eigen::Array2i& robotIndex, double resolution, double minLandA, Eigen::MatrixX4i& land_waypoints, int offset_w, int offset_h){
  int rows = mapdata.rows();
  int cols = mapdata.cols();
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
            if (mapdata(it_rows, it_cols)==100){
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
      for (int i=0;i<solutions;i++){
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
            }
          }
        }
      }
      Eigen::Matrix<int,1,4> temp;
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
      for(int i=0;i<w_div;i++){
        for(int j=0;j<h_div;j++){
          Eigen::MatrixXi new_submap = mapdata.block((sub_w*i)+offset_w, (sub_h*j)+offset_h, sub_w, sub_h);
          int new_offset_w = (sub_w*i)+offset_w;
          int new_offset_h = (sub_h*j)+offset_h;
          LandingCommander::splitncheck(new_submap, robotIndex, resolution, minLandA, land_waypoints, new_offset_w, new_offset_h);
        }
      }
    }
  }
  return true;
}


bool LandingCommander::toMatrix(const nav_msgs::OccupancyGrid& occupancyGrid, Eigen::MatrixXi& gridEigen){
  int height_ = occupancyGrid.info.height;
  int width_ = occupancyGrid.info.width;
  gridEigen.resize(height_,width_);
  // for (int i=0;i<occupancyGrid.data.size();i++) {
  //   Eigen::Array2i IndexXY; 
  //   IndexXY = getIndexFromLinearIndex(height_, width_, i);
  //   gridEigen(IndexXY(0),IndexXY(1))=occupancyGrid.data[i];
  // }
  for (std::vector<int8_t>::const_iterator it = occupancyGrid.data.cbegin(); it!=occupancyGrid.data.cend(); it++)
  return true;
}


bool LandingCommander::toOccupancyGrid(const Eigen::MatrixXi& gridEigen, nav_msgs::OccupancyGrid& occupancyGrid, const nav_msgs::OccupancyGrid& initialOccupancyGrid){
  int size = gridEigen.cols()*gridEigen.rows();
  occupancyGrid.data.resize(size);
  int k=0;
  for (int i=0; i<gridEigen.rows(); i++){
    for (int j=0; j<gridEigen.cols(); j++){
      occupancyGrid.data[k]=gridEigen(i,j);
      k++;
    }
  }
  occupancyGrid.info = initialOccupancyGrid.info;
  occupancyGrid.header = initialOccupancyGrid.header;

  return true;
}


int LandingCommander::checkNeighborsRadius (const double& safeAreaRadius, const double& resolution){
  int gridNumRadius;
  gridNumRadius = safeAreaRadius/resolution;
  return gridNumRadius;
}


Eigen::MatrixXi LandingCommander::checkEmMarkEm(Eigen::MatrixXi& matrix, int& radius){
  Eigen::MatrixXi newMatrix;
  newMatrix.resize(matrix.rows(),matrix.cols());
  int checkedSize = 0;
  for (int i=0;i<matrix.rows();i++){
    for (int j=0;j<matrix.cols();j++){
      if (matrix(i,j)==100){
        int subIndex_x, subIndex_y;
        int maxSubIndex_x,maxSubIndex_y;
        if (i-radius<0){subIndex_x=0;}
        else{subIndex_x = i-radius;}
        if (j-radius<0){subIndex_y=0;}
        else{subIndex_y = j-radius;}
        if (i+radius>matrix.rows()){maxSubIndex_x=matrix.rows();}
        else{maxSubIndex_x = i+radius;}
        if (j+radius>matrix.cols()){maxSubIndex_y=matrix.cols();}
        else{maxSubIndex_y = j+radius;}
        for (int k=subIndex_x;k<maxSubIndex_x;k++){
          for (int l=subIndex_y;l<maxSubIndex_y;l++){
            newMatrix(k,l)=100;
          }
        }
      }
    }
  }
  return newMatrix;
}


Eigen::Array2i LandingCommander::getIndexFromLinearIndex(int& height, int& width, int& Index){
  Eigen::Array2i IndexXY;
  int x = Index/width;
  int y = Index-(x*width);
  IndexXY(0) = x; IndexXY(1) = y;
  return IndexXY;
}

}