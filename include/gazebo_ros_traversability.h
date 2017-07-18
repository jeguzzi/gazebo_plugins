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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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
        void publishPoseAndTwist();
        ros::Time sim_time();

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
        ros::Publisher pose_publisher_, twist_publisher_;
        geometry_msgs::PoseStamped pose_msg_;
        geometry_msgs::TwistStamped twist_msg_;
        std::string robot_namespace_;
        std::vector<std::string> joint_names_;
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;
        bool use_ros_time_;
        bool publish_pose_twist_;
};

}

#endif
