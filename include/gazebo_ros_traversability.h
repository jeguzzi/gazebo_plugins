#ifndef GAZEBO_ROS_TRAVERSABILITY_HH
#define GAZEBO_ROS_TRAVERSABILITY_HH

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>

#include <sensor_msgs/JointState.h>
#include <gazebo_traversability_plugin/ContactState.h>
#include <gazebo_traversability_plugin/Power.h>

namespace gazebo
{

class GazeboRosTraversability : public ModelPlugin
{
public:
        /// \brief Constructor
        GazeboRosTraversability();
        /// \brief Destructor
        virtual ~GazeboRosTraversability();
        /// \brief Load the controller
        void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
        /// \brief Update the controller
        virtual void UpdateChild();

private:
        void OnUpdate ( const common::UpdateInfo & _info );
        void publishJointStates();
        void publishContacts();
        event::ConnectionPtr updateConnection;
        physics::WorldPtr world_;
        physics::ModelPtr parent_;
        physics::ContactManager *contact_manager_;
        std::vector<physics::JointPtr> joints_;
        std::map<physics::Collision *, unsigned char *> collisions_;
        boost::shared_ptr<ros::NodeHandle> rosnode_;
        sensor_msgs::JointState joint_msg_;
        gazebo_traversability_plugin::Power power_msg_;
        gazebo_traversability_plugin::ContactState contact_msg_;
        ros::Publisher joint_state_publisher_, power_publisher_, contact_publisher_;
        std::string robot_namespace_;
        std::vector<std::string> joint_names_;
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;
};

}

#endif
