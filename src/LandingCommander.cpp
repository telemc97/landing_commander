#include <landing_commander/LandingCommander.h>

using namespace grid_map;
namespace landing_commander{

LandingCommander::LandingCommander(const ros::NodeHandle &nh_)
: nodeHandle(nh_),
  gridMapSub(NULL),
  tfgridMapSub(NULL),
  baseFrameId("base_link"),
  safetyRadius(2),
  latchedTopics(true),
  minimumLandingArea(3.0)
  {
    nodeHandle.param("safety_dist", safetyRadius, safetyRadius);
    nodeHandle.param("robot_frame", baseFrameId, baseFrameId);
    nodeHandle.param("minimum_landing_area", minimumLandingArea, minimumLandingArea);

    occupancySub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("occupancy_output", 5, latchedTopics);

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
  geometry_msgs::Point robotPosMsg;
  tf::pointTFToMsg(robotOriginTf, robotPosMsg);
  toMatrix(*gridMap, OccupancyGridEigen);
  Eigen::Array2i robotPoseIndex;
  robotPoseIndex = (robotPosMsg.x, robotPosMsg.y);
  ROS_WARN("X: %i, Y: %i", robotPoseIndex(0), robotPoseIndex(1));
  double res = gridMap->info.resolution;

  for (int i=0;i<OccupancyGridEigen.rows();i++){
    for (int j=0;j<OccupancyGridEigen.cols();j++){
      if (OccupancyGridEigen(i,j)==100){
        for (int k=i-safetyRadius/2;k<safetyRadius||k<OccupancyGridEigen.rows();k++){
          for(int l=j-safetyRadius/2;l<safetyRadius||l<OccupancyGridEigen.cols();l++){
            OccupancyGridEigen(k,l)=100;
          }
        }
      }
    }
  }  

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
      Eigen::MatrixX4i temp;
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
  int width = occupancyGrid.info.width;
  int height = occupancyGrid.info.height;
  gridEigen.resize(width,height);
  for (std::vector<int8_t>::const_iterator iterator = occupancyGrid.data.begin(); iterator != occupancyGrid.data.end(); ++iterator) {
    for (int i=0; i<width; i++){
      for (int j=0; j<height; j++){
        gridEigen(i,j)=*iterator;
      }
    }
  }
  return true;
}

bool LandingCommander::toOccupancyGrid(const Eigen::MatrixXi& gridEigen, nav_msgs::OccupancyGrid& occupancyGrid, nav_msgs::OccupancyGrid initialOccupancyGrid){
  int width = gridEigen.rows();
  int height = gridEigen.cols();
  int size = width*height;
  if (initialOccupancyGrid.info.height != height && initialOccupancyGrid.info.width != width){
    ROS_WARN("shit went wrong");
  }
  occupancyGrid.info = initialOccupancyGrid.info;
  occupancyGrid.header = initialOccupancyGrid.header;
  occupancyGrid.data.resize(size);
  int k = 0;
  for (int i=0; i<width; i++){
    for (int j=0; j<height; j++){
      occupancyGrid.data[k] = gridEigen(i,j);
      k++;
    }
  }
    return true;
}

}