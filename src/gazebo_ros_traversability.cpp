#include <gazebo_ros_traversability.h>
#include <boost/algorithm/string.hpp>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosTraversability);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTraversability::GazeboRosTraversability()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTraversability::~GazeboRosTraversability()
{
        rosnode_->shutdown();
}


std::vector<std::string> readListParam(sdf::ElementPtr _sdf, std::string param)
{
        std::vector<std::string> ls;
        if ( !_sdf->HasElement ( param) ) {
                char msg[100];
                sprintf(msg, "GazeboRosTraversability Plugin missing %s", param.data());
                ROS_ASSERT (msg);
        } else {
                sdf::ElementPtr element = _sdf->GetElement ( param );
                std::string s = element->Get<std::string>();
                boost::erase_all ( s, " " );
                boost::split ( ls, s, boost::is_any_of ( "," ) );
        }
        return ls;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosTraversability::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
        // Make sure the ROS node for Gazebo has already been initalized
        if (!ros::isInitialized())
        {
                ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
        }

        // Store the pointer to the model
        parent_ = _parent;
        world_ = _parent->GetWorld();
        contact_manager_ = world_->GetPhysicsEngine()->GetContactManager();
        //contact_manager_->Init(this->world_);
        robot_namespace_ = parent_->GetName ();
        if ( !_sdf->HasElement ( "robotNamespace" ) ) {
                ROS_INFO_NAMED("gazebo_ros_traversability", "GazeboRosTraversability Plugin missing <robotNamespace>, defaults to \"%s\"",
                               robot_namespace_.c_str() );
        } else {
                robot_namespace_ = _sdf->GetElement ( "robotNamespace" )->Get<std::string>();
                if ( robot_namespace_.empty() ) robot_namespace_ = parent_->GetName ();
        }
        if ( !robot_namespace_.empty() ) robot_namespace_ += "/";
        rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( robot_namespace_ ) );

        update_rate_ = 100.0;
        if ( !_sdf->HasElement ( "updateRate" ) ) {
                ROS_WARN_NAMED("gazebo_ros_traversability",
                               "GazeboRosTraversability Plugin (ns = %s) missing <updateRate>, defaults to %f",
                               robot_namespace_.c_str(), update_rate_ );
        } else {
                update_rate_ = _sdf->GetElement ( "updateRate" )->Get<double>();
        }

        // Initialize update rate stuff
        if ( update_rate_ > 0.0 ) {
                update_period_ = 1.0 / update_rate_;
        } else {
                update_period_ = 0.0;
        }

        publish_pose_twist_ = false;

        if ( !_sdf->HasElement ( "publishPoseAndTwist" ) ) {
                ROS_WARN_NAMED("gazebo_ros_traversability",
                               "GazeboRosTraversability Plugin (ns = %s) missing <publishPoseAndTwist>, defaults to %d",
                               robot_namespace_.c_str(), publish_pose_twist_ );
        } else {
                publish_pose_twist_ = _sdf->GetElement ( "publishPoseAndTwist" )->Get<bool>();
        }

        use_ros_time_ = false;

        if ( !_sdf->HasElement ( "useROSTime" ) ) {
                ROS_WARN_NAMED("gazebo_ros_traversability",
                               "GazeboRosTraversability Plugin (ns = %s) missing <useROSTime>, defaults to %d",
                               robot_namespace_.c_str(), use_ros_time_ );
        } else {
                use_ros_time_ = _sdf->GetElement ( "useROSTime" )->Get<bool>();
        }

        joint_msg_.name = readListParam(_sdf, "jointName");
        uint n = joint_msg_.name.size();
        auto fields = {&joint_msg_.position, &joint_msg_.velocity, &joint_msg_.effort};
        for (auto field : fields)
        {
                field->resize(n);
        }
        for ( auto name : joint_msg_.name ) {
                physics::JointPtr joint =  this->parent_->GetJoint (name);
                joints_.push_back (joint);
                joint->SetProvideFeedback(true);
                ROS_INFO_NAMED("gazebo_ros_traversability", "GazeboRosTraversability is going to publish joint: %s", name.c_str() );
        }
        contact_msg_.name = readListParam(_sdf, "linkName");
        contact_msg_.contact.resize(contact_msg_.name.size());

        uint i = 0;
        for(auto name : contact_msg_.name)
        {
                uint index = 0;
                auto collision = parent_->GetLink(name)->GetCollision(index);
                collisions_[collision.get()] = &(contact_msg_.contact[i]);
                i++;
                ROS_INFO_NAMED("gazebo_ros_traversability", "GazeboRosTraversability is going to publish contacts for: %s", name.c_str() );
        }

        ROS_INFO_NAMED("gazebo_ros_traversability", "Starting GazeboRosTraversability Plugin (ns = %s)!, parent name: %s", this->robot_namespace_.c_str(), parent_->GetName ().c_str() );

        joint_state_publisher_ = rosnode_->advertise<sensor_msgs::JointState> ( "joint_states",0 );
        power_publisher_ = rosnode_->advertise<gazebo_traversability_plugin::Power> ( "power",0 );
        contact_publisher_ = rosnode_->advertise<gazebo_traversability_plugin::ContactState> ( "contact_states",0 );


        if(publish_pose_twist_)
        {
          pose_publisher_ = rosnode_->advertise<geometry_msgs::PoseStamped> ( "pose",0 );
          twist_publisher_ = rosnode_->advertise<geometry_msgs::TwistStamped> ( "twist",0 );
          pose_msg_.header.frame_id = "gz";
          twist_msg_.header.frame_id = "gz";
        }

        last_update_time_ = this->world_->GetSimTime();
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin (
                boost::bind ( &GazeboRosTraversability::OnUpdate, this, _1 ) );

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTraversability::UpdateChild()
{
}


void GazeboRosTraversability::OnUpdate ( const common::UpdateInfo & _info ) {
        common::Time current_time = world_->GetSimTime();
        double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
        if ( seconds_since_last_update > update_period_ ) {
                publishJointStates();
                publishContacts();
                if(publish_pose_twist_)
                {
                  publishPoseAndTwist();
                }
                last_update_time_+= common::Time ( update_period_ );
        }
}

void GazeboRosTraversability::publishPoseAndTwist()
{

    ros::Time stamp = sim_time();

    math::Pose pose = parent_->GetWorldPose();
    pose_msg_.pose.position.x = pose.pos.x;
    pose_msg_.pose.position.y = pose.pos.y;
    pose_msg_.pose.position.z = pose.pos.z;
    pose_msg_.pose.orientation.x = pose.rot.x;
    pose_msg_.pose.orientation.y = pose.rot.y;
    pose_msg_.pose.orientation.z = pose.rot.z;
    pose_msg_.pose.orientation.w = pose.rot.w;

    pose_msg_.header.stamp = stamp;

    math::Vector3 linear, angular;
    linear = parent_->GetWorldLinearVel();
    angular = parent_->GetWorldAngularVel();
    twist_msg_.twist.linear.x = linear.x;
    twist_msg_.twist.linear.y = linear.y;
    twist_msg_.twist.linear.z = linear.z;
    twist_msg_.twist.angular.x = angular.x;
    twist_msg_.twist.angular.y = angular.y;
    twist_msg_.twist.angular.z = angular.z;

    twist_msg_.header.stamp = stamp;

    pose_publisher_.publish(pose_msg_);
    twist_publisher_.publish(twist_msg_);

}


void GazeboRosTraversability::publishContacts()
{
        unsigned int n = this->contact_manager_->GetContactCount();
        for(auto &pair : collisions_)
        {
                *(pair.second) = 0;
        }

        uint i=0;
        for(auto contact : this->contact_manager_->GetContacts ())
        {
                if(i++==n) break;
                physics::Collision *c1 = contact->collision1;
                physics::Collision *c2 = contact->collision2;
                if(collisions_.count(c1)) *(collisions_[c1]) = 1;
                if(collisions_.count(c2)) *(collisions_[c2]) = 1;
        }
        contact_msg_.header.stamp = sim_time();
        contact_publisher_.publish(contact_msg_);
}


void GazeboRosTraversability::publishJointStates() {

        ros::Time current_time = sim_time();
        joint_msg_.header.stamp = current_time;
        for ( int i = 0; i < joints_.size(); i++ ) {
                physics::JointPtr joint = joints_[i];
                joint_msg_.position[i] = joint->GetAngle ( 0 ).Radian ();
                double omega = joint->GetVelocity (0);
                joint_msg_.velocity[i] = omega;
                math::Vector3 axis = joint->GetGlobalAxis(0);
                math::Vector3 v_torque = joint->GetLinkTorque(0);
                double torque = axis.Dot(v_torque);
                joint_msg_.effort[i] = torque;
        }
        joint_state_publisher_.publish ( joint_msg_ );
        //We assume that joints are FL, FR, BL, BR
        double power_left_motor = (joint_msg_.effort[0] * joint_msg_.velocity[0] + joint_msg_.effort[2] * joint_msg_.velocity[2]);
        double power_right_motor = (joint_msg_.effort[1] * joint_msg_.velocity[1] + joint_msg_.effort[3] * joint_msg_.velocity[3]);
        double power = fmax(0, power_left_motor) + fmax(0, power_right_motor);

        power_msg_.header.stamp = current_time;
        power_msg_.power = power;
        power_msg_.energy += power * update_period_;
        power_publisher_.publish(power_msg_);
}

ros::Time GazeboRosTraversability::sim_time()
{
  if(use_ros_time_)
  {
    return ros::Time::now();
  }
  else
  {
    common::Time current_time = world_->GetSimTime();
    return ros::Time(current_time.sec, current_time.nsec);
  }
}

}
