#include <landing_commander/LandingCommander.h>

using namespace grid_map;
namespace landing_commander{

LandingCommander::LandingCommander(const ros::NodeHandle &nh_)
: nodeHandle(nh_),
  gridMapSub(NULL),
  tfgridMapSub(NULL),
  baseFrameId("base_link"),
  safetyRadius(2.0),
  latchedTopics(true)
  {
    nodeHandle.param("safety_dist", safetyRadius, safetyRadius);
    nodeHandle.param("robot_frame", baseFrameId, baseFrameId);

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
  Eigen::Vector3d robotPoseEigen;
  tf::pointMsgToEigen(robotPosMsg, robotPoseEigen);
  Eigen::Vector2d robotPoseEigen2d = robotPoseEigen.head(2);
  GridMapRosConverter::fromOccupancyGrid(*gridMap, "occupancy_grid", gridMapConverted);
  Index robotPoseIndex;
  gridMapConverted.getIndex(robotPoseEigen2d, robotPoseIndex);
  Matrix& mapdata = gridMapConverted["occupancy_grid"];
  for (GridMapIterator iterator0(gridMapConverted); !iterator0.isPastEnd(); ++iterator0){
    if (gridMapConverted.at("occupancy_grid", *iterator0) == 100){
      Position centerPos;
      gridMapConverted.getPosition(*iterator0, centerPos);
      for (CircleIterator iterator1(gridMapConverted, centerPos, safetyRadius); !iterator1.isPastEnd(); ++iterator1){
        gridMapConverted.at("occupancy_grid", *iterator1) = 1;
      }
    }
  }

  Eigen::MatrixXi land_waypoints =  LandingCommander::splitncheck(mapdata, robotPoseIndex);

  GridMapRosConverter::toOccupancyGrid(gridMapConverted, "occupancy_grid", 0, 1, gridMapOutput);
  occupancySub.publish(gridMapOutput);
}

Eigen::MatrixXi LandingCommander::splitncheck(const Eigen::MatrixXi& mapdata, Eigen::Array2i& robotIndex, int offset_w = 0, int offset_h = 0){
  int rows = mapdata.rows();
  int cols = mapdata.cols();
  int w_div = LandingCommander::maxDivider(rows);
  int h_div = LandingCommander::maxDivider(cols);
  int submaps = w_div*h_div;
  Eigen::MatrixXi score(w_div,h_div);
  int sub_w = rows/w_div;
  int sub_h = cols/h_div;
  for (int i=0; i<w_div; i++){
    for (int j=0; j<h_div; j++){
      int occ_cells = 0;
      for (int it_rows=sub_w*w_div+offset_w; it_rows<sub_w+sub_w*w_div+offset_w; it_rows++){
        for (int it_cols=sub_h*h_div+offset_h; it_cols<sub_h+sub_h*h_div+offset_h; it_cols++){
          if (mapdata(it_rows, it_cols)==100){
            occ_cells++;
          }
        }
      }
      score(w_div,h_div) = occ_cells;
    }
  }
  int best = score(0,0);
  int solutions = 0;
  for (int i=0; i<w_div;i++){
    for (int j=0;j<h_div;j++){
      if (score(w_div,h_div)<best){
        best = score(w_div,h_div);
      }
      if (score(w_div,h_div)==0){solutions++;}
    }
  }
  if (best==0){
    Eigen::MatrixXi land_waypoints;
    for (int i=0;i<solutions;i++){
      for (int j=0;j<w_div;j++){
        for (int k=0;k<h_div;k++){
          if (score(w_div,h_div)==0){
            int centroid_w = (sub_w/2)+(sub_w*w_div)+offset_w;
            int centroid_h = (sub_h/2)+(sub_h*h_div)+offset_h;
            int dist = std::sqrt(std::pow((robotIndex(0)-centroid_w), 2)+std::pow((robotIndex(1)-centroid_h), 2));
            land_waypoints(i,0) = centroid_w;
            land_waypoints(i,1) = centroid_h;
            land_waypoints(i,2) = dist;
          }
        }
      }
    }
    Eigen::Matrix3i temp;
    for (int i=0;i<solutions;i++){
      for (int j=i+1;j<solutions;j++){
        if (land_waypoints(i,2)>land_waypoints(j,2)){
          temp = land_waypoints.block(i,0,1,3);
          land_waypoints.block(i,0,1,3) = land_waypoints.block(j,0,1,3);
          land_waypoints.block(j,0,1,3) = temp;
        }
      }
    }
  }else{
    for(int i=0;i<w_div;i++){
      for(int j=0;j<h_div;j++){
        Eigen::MatrixXf new_submap = mapdata.block((sub_w*w_div)+offset_w, (sub_h*h_div)+offset_h, sub_w, sub_h);
        int new_offset_w = (sub_w*w_div)+offset_w;
        int new_offset_h = (sub_h*h_div)+offset_h;
        LandingCommander::splitncheck(new_submap, robotIndex, new_offset_w, new_offset_h);
      }
    }
  }
}



}