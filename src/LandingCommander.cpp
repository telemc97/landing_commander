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
  GridMapRosConverter::fromOccupancyGrid(*gridMap, "occupancy_grid", gridMapConverted);
  Matrix& data = gridMapConverted["occupancy_grid"];
  for (GridMapIterator iterator0(gridMapConverted); !iterator0.isPastEnd(); ++iterator0){
    if (gridMapConverted.at("occupancy_grid", *iterator0) == 100){
      Position centerPos;
      gridMapConverted.getPosition(*iterator0, centerPos);
      for (CircleIterator iterator1(gridMapConverted, centerPos, safetyRadius); !iterator1.isPastEnd(); ++iterator1){
        gridMapConverted.at("occupancy_grid", *iterator1) = 1;
      }
    }
  }
  GridMapRosConverter::toOccupancyGrid(gridMapConverted, "occupancy_grid", 0, 1, gridMapOutput);
  occupancySub.publish(gridMapOutput);
}


}