#ifndef GAZEBO_ROS_POWER_HH
#define GAZEBO_ROS_POWER_HH

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

namespace gazebo
{

   class GazeboRosPower : public ModelPlugin
   {
      /// \brief Constructor
      public: GazeboRosPower();

      /// \brief Destructor
      public: virtual ~GazeboRosPower();

      /// \brief Load the controller
      public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
      void OnUpdate ( const common::UpdateInfo & _info );
      void publishJointStates();
      /// \brief Update the controller
      protected: virtual void UpdateChild();

    private:
      event::ConnectionPtr updateConnection;
physics::WorldPtr world_;
physics::ModelPtr parent_;
std::vector<physics::JointPtr> joints_;

// ROS STUFF
boost::shared_ptr<ros::NodeHandle> rosnode_;
sensor_msgs::JointState joint_state_;
ros::Publisher joint_state_publisher_, power_publisher_;
std::string tf_prefix_;
std::string robot_namespace_;
std::vector<std::string> joint_names_;

// Update Rate
double update_rate_;
double update_period_;
common::Time last_update_time_;

   };

}

#endif
