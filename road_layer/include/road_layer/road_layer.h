#ifndef ROAD_LAYER_H_
#define ROAD_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <nav_msgs/OccupancyGrid.h>

#include <nav_msgs/OccupancyGrid.h>

#include <costmap_2d/ObstaclePluginConfig.h>
#include <costmap_2d/footprint.h>
#include <sensor_msgs/PointCloud.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

#include <costmap_2d/footprint.h>

namespace road_layer_namespace
{

class RoadLayer : public costmap_2d::Layer
{
public:
  RoadLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);

  double mark_x_, mark_y_;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
}
#endif
