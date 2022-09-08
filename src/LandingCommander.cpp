#include <landing_commander/LandingCommander.h>

using namespace grid_map;
namespace landing_commander{

LandingCommander::LandingCommander(const ros::NodeHandle &nh_)
: nodeHandle(nh_),
  gridMapSub(NULL),
  tfgridMapSub(NULL),
  baseFrameId("base_link"),
  safetyRange(1.0),
  latchedTopics(true)
  {
    nodeHandle.param("safety_dist", safetyRange, safetyRange);
    nodeHandle.param("robot_frame", baseFrameId, baseFrameId);

    occupancySub = nodeHandle.advertise<nav_msgs::OccupancyGrid>("occupancy_output", 1, latchedTopics);

    gridMapSub = new message_filters::Subscriber<nav_msgs::OccupancyGrid>(nodeHandle, "/projected_map", 1);
    tfgridMapSub = new tf::MessageFilter<nav_msgs::OccupancyGrid>(*gridMapSub, tfListener, baseFrameId, 3);
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
  GridMapRosConverter::fromOccupancyGrid(*gridMap, "traversability", gridMapConverted);
  Matrix& data = gridMapConverted["traversability"];
  for (GridMapIterator iterator0(gridMapConverted); !iterator0.isPastEnd(); ++iterator0){
    const int i = iterator0.getLinearIndex();
    if (data(i)==100){
      Position centerPos;
      gridMapConverted.getPosition(*iterator0, centerPos);
      for (CircleIterator iterator1(gridMapConverted, centerPos, safetyRange); !iterator1.isPastEnd(); ++iterator1){
        gridMapConverted.at("traversability", *iterator1) = 100;
      }
    }
  }
  GridMapRosConverter::toOccupancyGrid(gridMapConverted, "traversability", 0, 100, gridMapOutput);
  occupancySub.publish(gridMapOutput);
}


}