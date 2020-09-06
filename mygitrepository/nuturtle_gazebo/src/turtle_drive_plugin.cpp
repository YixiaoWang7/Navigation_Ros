#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <nuturtlebot/WheelCommands.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <nuturtlebot/SensorData.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
    class DiffModel : public ModelPlugin
    {
        public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            if (!ros::isInitialized())
            {
                ROS_FATAL("A ROS node for Gazebo has not been initialized."
                            "Unable to load plugin. Load the Gazebo system plugin"
                            "'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
                return;
            }
            // Store the pointer to the model
            this->model = _model;

            if (!_sdf->HasElement("left_wheel_joint"))
            {
                ROS_INFO("fata error. There is no parameter named left_wheel_joint.");
                return;
            }
            if (!_sdf->HasElement("right_wheel_joint"))
            {
                ROS_INFO("fata error. There is no parameter named right_wheel_joint.");
                return;
            }
            if (!_sdf->HasElement("wheel_cmd_topic"))
            {
                ROS_INFO("fata error. There is no parameter named wheel_cmd_topic.");
                return;
            }
            if (!_sdf->HasElement("sensor_data_topic")) 
            {
                ROS_INFO("fata error. There is no parameter named sensor_data_topic.");
                return;
            }
            if (!_sdf->HasElement("sensor_frequency"))
            {
                this->sensor_frequency=200.0;
            }
            this->left_wheel_joint=_sdf->GetElement("left_wheel_joint")->Get<std::string>();
            this->right_wheel_joint=_sdf->GetElement("right_wheel_joint")->Get<std::string>();
            this->wheel_cmd_topic=_sdf->GetElement("wheel_cmd_topic")->Get<std::string>();
            this->sensor_data_topic=_sdf->GetElement("sensor_data_topic")->Get<std::string>();
            this->sensor_frequency=_sdf->GetElement("sensor_frequency")->Get<double>();

            this->n.getParam("/max_power_motor",this->max_power_motor);
            this->n.getParam("/max_rot_v_motor",this->max_rot_v_motor);
            this->n.getParam("/encoder_ticks_per_rev",this->encoder_ticks_per_rev);

            // // whether the parameters are loaded properly
            // ROS_INFO("left:%s",this->left_wheel_joint.c_str());
            // ROS_INFO("Hz: %f",this->sensor_frequency);
            this->joints.resize(2);
            this->joints[0] = this->model->GetJoint(this->left_wheel_joint);

            if (!this->joints[0])
            {
                ROS_INFO("cannot find %s in sdf file.",this->left_wheel_joint.c_str());
                return;
            }
            this->joints[1] = this->model->GetJoint(this->right_wheel_joint);
            if (!this->joints[1])
            {
                ROS_INFO("cannot find %s in sdf file.",this->right_wheel_joint.c_str());
                return;
            }
            
            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node

            this->wheel_cmd_sub = this->n.subscribe(wheel_cmd_topic,1,&DiffModel::wheelCmdBack,this);
            this->wheel_cmd.left_velocity=0.0;
            this->wheel_cmd.right_velocity=0.0;
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            
            this->sensor_data_pub=this->n.advertise<nuturtlebot::SensorData>("sensor_data",1);
            this->pub_timer=this->n.createTimer(ros::Duration(1.0/this->sensor_frequency),&DiffModel::sensorDataPub,this);

            this->sensor_data.accelX=0;
            this->sensor_data.accelY=0;
            this->sensor_data.accelZ=0;
            this->sensor_data.battery_voltage=0;
            this->sensor_data.gyroX=0;
            this->sensor_data.gyroY=0;
            this->sensor_data.gyroZ=0;
            this->sensor_data.left_encoder=0;
            this->sensor_data.magX=0;
            this->sensor_data.magY=0;
            this->sensor_data.magZ=0;
            this->sensor_data.right_encoder=0;
            this->sensor_data.stamp=ros::Time::now();
            ROS_INFO("load and initialize sucessfully.");
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&DiffModel::OnUpdate, this));
        };

        // Called by the world update start event
        public: void OnUpdate()
        {
            // this->joints[0]->SetVelocityLimit(0, this->max_rot_v_motor);
            // this->joints[0]->SetVelocity(0, this->max_rot_v_motor/256.0*this->wheel_cmd.left_velocity);
            // this->joints[1]->SetVelocityLimit(0, this->max_rot_v_motor);
            // this->joints[1]->SetVelocity(0, this->max_rot_v_motor/256.0*this->wheel_cmd.right_velocity);
            double w;
            w=this->max_rot_v_motor/256.0*this->wheel_cmd.left_velocity;
            if (fabs(w)<1e-6)
            {
                this->joints[0]->SetParam("fmax",0,1.4);
            }else
            {
                this->joints[0]->SetParam("fmax",0,max_power_motor/w);
            }
            this->joints[0]->SetParam("vel",0,w);


            w=this->max_rot_v_motor/256.0*this->wheel_cmd.right_velocity;
            if (fabs(w)<1e-6)
            {
                this->joints[1]->SetParam("fmax",0,1.4);
            }else
            {
                this->joints[1]->SetParam("fmax",0,max_power_motor/w);
            }
            this->joints[1]->SetParam("vel",0,w);

            this->sensor_data.stamp=ros::Time::now();
            this->sensor_data.left_encoder=this->joints[0]->Position(0)/2.0/this->PI*this->encoder_ticks_per_rev;
            this->sensor_data.right_encoder=this->joints[1]->Position(0)/2.0/this->PI*this->encoder_ticks_per_rev;
            
            // // Apply a small linear velocity to the model.
            // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
        };
        
        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        public: void wheelCmdBack(const nuturtlebot::WheelCommands::ConstPtr &_msg)
        {
            wheel_cmd.left_velocity=_msg->left_velocity;
            wheel_cmd.right_velocity=_msg->right_velocity;
        }

        /// \brief Handle an incoming message from ROS
        /// \param[in] _msg A float value that is used to set the velocity
        /// of the Velodyne.
        public: void sensorDataPub(const ros::TimerEvent& event)
        {
            this->sensor_data_pub.publish(this->sensor_data);
        }

        private: std::string left_wheel_joint, right_wheel_joint, wheel_cmd_topic, sensor_data_topic;

        private: double sensor_frequency, max_rot_v_motor=6.35492, max_power_motor=20.0, encoder_ticks_per_rev=4096;

        private: nuturtlebot::WheelCommands wheel_cmd;
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        /// \brief A node use for ROS transport
        private: ros::NodeHandle n;
        
        /// \brief a vector containing the joints ptr
        private: std::vector<physics::JointPtr> joints;

        /// \brief a physical model
        private: physics::ModelPtr model;

        /// \brief A ROS subscriber
        private: ros::Subscriber wheel_cmd_sub;

        /// \brief joint states:left and right
        private: double position_left=0, position_right=0;

        /// \brief A ROS subscriber
        private: ros::Publisher sensor_data_pub;

        private: ros::Timer pub_timer;

        private: nuturtlebot::SensorData sensor_data;

        private: double PI=3.14159265359;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(DiffModel)
}