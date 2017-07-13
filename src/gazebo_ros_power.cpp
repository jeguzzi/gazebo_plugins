#include <gazebo_ros_power.h>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosPower);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosPower::GazeboRosPower()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosPower::~GazeboRosPower()
{
  rosnode_->shutdown();
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosPower::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  // Make sure the ROS node for Gazebo has already been initalized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }



  // Store the pointer to the model
  this->parent_ = _parent;
  this->world_ = _parent->GetWorld();
  this->robot_namespace_ = parent_->GetName ();
      if ( !_sdf->HasElement ( "robotNamespace" ) ) {
          ROS_INFO_NAMED("gazebo_ros_power", "GazeboRosPower Plugin missing <robotNamespace>, defaults to \"%s\"",
                     this->robot_namespace_.c_str() );
      } else {
          this->robot_namespace_ = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();
          if ( this->robot_namespace_.empty() ) this->robot_namespace_ = parent_->GetName ();
      }
      if ( !robot_namespace_.empty() ) this->robot_namespace_ += "/";
      rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( this->robot_namespace_ ) );

      if ( !_sdf->HasElement ( "jointName" ) ) {
          ROS_ASSERT ( "GazeboRosPower Plugin missing jointNames" );
      } else {
          sdf::ElementPtr element = _sdf->GetElement ( "jointName" ) ;
          std::string joint_names = element->Get<std::string>();
          boost::erase_all ( joint_names, " " );
          boost::split ( joint_names_, joint_names, boost::is_any_of ( "," ) );
      }

      this->update_rate_ = 100.0;
      if ( !_sdf->HasElement ( "updateRate" ) ) {
          ROS_WARN_NAMED("joint_state_publisher", "GazeboRosPower Plugin (ns = %s) missing <updateRate>, defaults to %f",
                     this->robot_namespace_.c_str(), this->update_rate_ );
      } else {
          this->update_rate_ = _sdf->GetElement ( "updateRate" )->Get<double>();
      }

      // Initialize update rate stuff
      if ( this->update_rate_ > 0.0 ) {
          this->update_period_ = 1.0 / this->update_rate_;
      } else {
          this->update_period_ = 0.0;
      }
      last_update_time_ = this->world_->GetSimTime();

      for ( unsigned int i = 0; i< joint_names_.size(); i++ ) {
          physics::JointPtr joint =  this->parent_->GetJoint ( joint_names_[i] );
          joints_.push_back (joint);
          joint->SetProvideFeedback(true);
          ROS_INFO_NAMED("gazebo_ros_power", "GazeboRosPower is going to publish joint: %s", joint_names_[i].c_str() );
      }



      ROS_INFO_NAMED("gazebo_ros_power", "Starting GazeboRosPower Plugin (ns = %s)!, parent name: %s", this->robot_namespace_.c_str(), parent_->GetName ().c_str() );

      tf_prefix_ = tf::getPrefixParam ( *rosnode_ );
      joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState> ( "joint_states",1000 );
      power_publisher_ = rosnode_->advertise<std_msgs::Float32> ( "power",1000 );

      last_update_time_ = this->world_->GetSimTime();
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin (
                                   boost::bind ( &GazeboRosPower::OnUpdate, this, _1 ) );

      printf("DONE\n");
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosPower::UpdateChild()
{
}




void GazeboRosPower::OnUpdate ( const common::UpdateInfo & _info ) {
    // Apply a small linear velocity to the model.
    common::Time current_time = this->world_->GetSimTime();
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {

        publishJointStates();

        last_update_time_+= common::Time ( update_period_ );

    }

}

void GazeboRosPower::publishJointStates() {
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );
    joint_state_.velocity.resize ( joints_.size() );
    joint_state_.effort.resize ( joints_.size() );
    float power = 0;
    //float power2 = 0;
    for ( int i = 0; i < joints_.size(); i++ ) {
        physics::JointPtr joint = joints_[i];
        math::Angle angle = joint->GetAngle ( 0 );
        joint_state_.name[i] = joint->GetName();
        joint_state_.position[i] = angle.Radian () ;


        double omega = joint->GetVelocity (0) ;

        joint_state_.velocity[i] = omega ;


        //double f = joint->GetForce();


        math::Vector3 axis = joint->GetGlobalAxis(0);
        math::Vector3 torque2 = joint->GetLinkTorque(0);
        //math::Vector3 torque3 = joint->GetLinkTorque(1);
        // const char *name0 = joint->GetJointLink(0)->GetName().data();
        // const char *name1 = joint->GetJointLink(1)->GetName().data();

        //printf("Joint %d: Link 0 %s, Link 1 %s\n", i, name0, name1);

        double torque = axis.Dot(torque2);

        //double joint_power = torque * omega;
        //double joint_power1 = - axis.Dot(torque3) * omega;

        //printf("joint %d: axis %f %f %f, torque %f %f %f, omega %f, power %f, power e %f\n", i, axis[0], axis[1], axis[2], torque2[0], torque2[1], torque2[2], omega, joint_power, joint_power1);


        //physics::JointWrench wrench = joint->GetForceTorque(0);

        //math::Vector3 torque = wrench.body1Torque;//joint->GetLinkTorque (0);
        //math::Vector3 torque1 = wrench.body2Torque;//joint->GetLinkTorque (1);
        //printf("joint %d torque: [%f, %f %f] \n", i, torque[0], torque[1], torque[2]);
        //printf("joint %d torque1: [%f, %f %f] \n", i, torque1[0], torque1[1], torque1[2]);
        joint_state_.effort[i] = torque ;
        // power += fmax(0, joint_power);
        // power2 += fmax(0, joint_power1);
    }

    double power_left_motor = (joint_state_.effort[0] * joint_state_.velocity[0] + joint_state_.effort[2] * joint_state_.velocity[2]);
    double power_right_motor = (joint_state_.effort[1] * joint_state_.velocity[1] + joint_state_.effort[3] * joint_state_.velocity[3]);

    power = fmax(0, power_left_motor) + fmax(0, power_right_motor);

    joint_state_publisher_.publish ( joint_state_ );
    std_msgs::Float32 msg;
    msg.data = power;
    power_publisher_.publish(msg);

    //printf("Power %f (-> %f)\n", power, power2);

}

}
