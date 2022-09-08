#include <landing_commander/LandingCommander.h>

using namespace grid_map;
namespace landing_commander{

LandingCommander::LandingCommander(const ros::NodeHandle &nh_)
: nodeHandle(nh_),
  gridMapSub(NULL),
  tfgridMapSub(NULL),
  baseFrameId("base_link"),
  safetyRange(5.0)
  {
    nodeHandle.param("safety_dist", safetyRange, safetyRange);
    nodeHandle.param("robot_frame", baseFrameId, baseFrameId);
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
    const Index index(*iterator0);
    if (data(index(0), index(1))=100){
      Index centerIndex(index(0), index(1));
      Position centerPos;
      gridMapConverted.getPosition(centerIndex, centerPos);
      for (CircleIterator iterator1(gridMapConverted, centerPos, safetyRange); !iterator1.isPastEnd(); ++iterator1){
        data(index(0), index(1))==100;
      }
    }
  }
  
  
}


}