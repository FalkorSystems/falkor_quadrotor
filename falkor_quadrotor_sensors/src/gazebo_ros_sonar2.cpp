#include <falkor_quadrotor_sensors/gazebo_ros_sonar2.h>
#include "common/Events.hh"
#include "physics/physics.h"

#include <limits>

namespace gazebo {

GazeboRosSonar2::GazeboRosSonar2()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSonar2::~GazeboRosSonar2()
{
  event::Events::DisconnectWorldUpdateStart(updateConnection);
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
  void GazeboRosSonar2::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if( !_sdf->HasElement("remoteModelName"))
    remote_model_name_ = "robot";
  else
    remote_model_name_ = _sdf->GetElement("remoteModelName")->GetValueString();

  remote_model = boost::shared_dynamic_cast<physics::Model>(world->GetEntity(remote_model_name_));
  if( !remote_model )
    ROS_FATAL( "gazebo_ros_sonar2: unable to get remote model: %s", remote_model_name_.c_str() );

  ROS_DEBUG_NAMED( "gazebo_ros_sonar2", "got remote model: %s", remote_model->GetName().c_str() );

  if( !_sdf->HasElement("remoteBodyName"))
  {
    remote_link = remote_model->GetLink();
    remote_link_name_ = remote_link->GetName();
  }
  else
  {
    remote_link_name_ = _sdf->GetElement("remoteBodyName")->GetValueString();
    remote_link = remote_model->GetLink( remote_link_name_ );
  }

  if( !remote_link )
    ROS_FATAL( "gazebo_ros_sonar2: unable to get remote link: %s",
	       remote_link_name_.c_str() );
    
  ROS_DEBUG_NAMED( "gazebo_ros_sonar2", "got remote link: %s", remote_link->GetName().c_str() );

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValueString();
    link = _model->GetLink( link_name_ );
  }
  
  ROS_DEBUG_NAMED( "gazebo_ros_sonar2", "got local link: %s", link->GetName().c_str() );

  if (!_sdf->HasElement("frameId"))
    frame_id_ = "";
  else
    frame_id_ = _sdf->GetElement("frameId")->GetValueString();

  if (!_sdf->HasElement("topicName"))
    topic_ = "sonar";
  else
    topic_ = _sdf->GetElement("topicName")->GetValueString();

  range_.header.frame_id = frame_id_;
  range_.radiation_type = sensor_msgs::Range::ULTRASOUND;

  if( !_sdf->HasElement( "rangeMax" ) )
    range_.max_range = 100;
  else
    range_.max_range = _sdf->GetElement("rangeMax")->GetValueDouble();

  if( !_sdf->HasElement( "rangeMin" ) )
    range_.min_range = 1;
  else
    range_.min_range = _sdf->GetElement("rangeMin")->GetValueDouble();

  sensor_model_.Load( _sdf );

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  publisher_ = node_handle_->advertise<sensor_msgs::Range>(topic_, 1);

  Reset();
  updateConnection = event::Events::ConnectWorldUpdateStart( boost::bind( &GazeboRosSonar2::Update, this ));
}

void GazeboRosSonar2::Reset()
{
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSonar2::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();

  range_.header.stamp.sec  = (world->GetSimTime()).sec;
  range_.header.stamp.nsec = (world->GetSimTime()).nsec;

  // get the distance between the model and the remote_model
  math::Vector3 pos = link->GetWorldPose().pos;
  math::Vector3 remote_pos = remote_link->GetWorldPose().pos;

  range_.range = pos.Distance( remote_pos );
 
  ROS_DEBUG_NAMED( "gazebo_ros_sonar2", "range: %g", range_.range );

  // add Gaussian noise (and limit to min/max range)
  if (range_.range < range_.max_range) {
    range_.range += sensor_model_.update(dt);
    if (range_.range < range_.min_range) range_.range = range_.min_range;
    if (range_.range > range_.max_range) range_.range = range_.max_range;
  }

  publisher_.publish(range_);

  // save last time stamp
  last_time = sim_time;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosSonar2)

} // namespace gazebo
