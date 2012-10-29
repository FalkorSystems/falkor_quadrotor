#ifndef FALKOR_QUADROTOR_SENSORS_GAZEBO_ROS_SONAR2_H
#define FALKOR_QUADROTOR_SENSORS_GAZEBO_ROS_SONAR2_H

#include "common/Plugin.hh"

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <sensor_msgs/Range.h>
#include <hector_gazebo_plugins/sensor_model.h>

namespace gazebo
{

class GazeboRosSonar2 : public ModelPlugin
{
public:
  GazeboRosSonar2();
  virtual ~GazeboRosSonar2();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  physics::ModelPtr remote_model;

  physics::LinkPtr remote_link;
  physics::LinkPtr link;

  ros::NodeHandle* node_handle_;
  ros::Publisher publisher_;

  sensor_msgs::Range range_;

  std::string namespace_;
  std::string topic_;
  std::string frame_id_;
  std::string remote_model_name_;
  std::string remote_link_name_;
  std::string link_name_;

  SensorModel sensor_model_;

  /// \brief save last_time
  common::Time last_time;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

} // namespace gazebo

#endif // FALKOR_QUADROTOR_SENSORS_GAZEBO_ROS_SONAR2_H
