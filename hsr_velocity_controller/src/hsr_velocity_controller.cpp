#include <vector>
#include <string>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <urdf/model.h>

namespace hsr_velocity_controller_ns{

    class HsrVelocityController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
    {
        bool init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle &n)
        {
            // List of controlled joints
            std::string param_name = "joints";
            if(!n.getParam(param_name, joint_names_))
            {
                ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
                return false;
            }
            n_joints_ = joint_names_.size();

            if(n_joints_ == 0){
                ROS_ERROR_STREAM("List of joint names is empty.");
                return false;
            }
            for(unsigned int i=0; i<n_joints_; i++)
            {
                try
                {
                    joints_.push_back(hw->getHandle(joint_names_[i]));
                }
                catch (const hardware_interface::HardwareInterfaceException& e)
                {
                    ROS_ERROR_STREAM("Exception thrown: " << e.what());
                    return false;
                }
            }

            commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

            sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &HsrVelocityController::commandCB, this);
            // pub_ = n.advertise<std_msgs::Float64MultiArray>("controller_state", 1);
            pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(n, "controller_state", 1));
            urdf::Model urdf;
            if (!urdf.initParamWithNodeHandle("robot_description", n))
            {
                ROS_ERROR("Failed to parse urdf file");
                return false;
            }
            
            counter = 0;
            js_ = std::vector<double>(n_joints_);
            joints_urdf_ = std::vector<urdf::JointConstSharedPtr>(n_joints_);
            for(unsigned int i ; i<n_joints_; i++)
            {
                js_[i] = joints_[i].getPosition();
                joints_urdf_[i] = urdf.getJoint(joint_names_[i]);
                ROS_ERROR_STREAM("Joint " << joint_names_[i] << " lower limit " << joints_urdf_[i]->limits->lower << " upper limit " << joints_urdf_[i]->limits->upper);
            }
            
            n.getParam("p_gains", p_gains_);
            n.getParam("i_gains", i_gains_);
            n.getParam("d_gains", d_gains_);
            n.getParam("feedforward_gains", ff_gains_);

            old_integrator_ = std::vector<double>(n_joints_);
            old_error_ = std::vector<double>(n_joints_);
            filtered_vel_ = std::vector<double>(n_joints_);

            return true;
        }

        void starting(const ros::Time& time){}
        void stopping(const ros::Time& time){}

        void update(const ros::Time& time, const ros::Duration& period)
        {
            std::vector<double> & commands = *commands_buffer_.readFromRT();
            std::vector<double>  d(n_joints_+1, 0);
            
            double dt = period.toNSec() / 1e9;
            for(unsigned int i = 0; i<n_joints_; i++) // the problem was that the i was not initialized...
            {
                double vel_cmd = commands[i];
                // filter the velocity
                float alpha = 0.95f;
                filtered_vel_[i] = (1.0f - alpha) * filtered_vel_[i] + alpha * joints_[i].getVelocity();
                if(vel_cmd == 0.0)
                {
                    js_[i] = joints_[i].getPosition();
                    old_integrator_[i] = 0.0;
                    old_error_[i] = 0.0;
                
                }else
                {
                    double error = (vel_cmd - filtered_vel_[i]);
                    double new_integrator = old_integrator_[i] + error * dt;
                    
                    double next_pos = joints_[i].getPosition() + vel_cmd * dt * ff_gains_[i] 
                                        + error * p_gains_[i] 
                                        + new_integrator * i_gains_[i] 
                                        + d_gains_[i]/dt * (error - old_error_[i]);

                    // clamp the output by the joint limits and disable further integration integration
                    if(next_pos > joints_urdf_[i]->limits->upper)
                    {
                        next_pos = joints_urdf_[i]->limits->upper;
                        if(error < 0)
                        {
                            old_integrator_[i] = new_integrator;
                        }
                    }
                    else if(next_pos < joints_urdf_[i]->limits->lower)
                    {
                        next_pos = joints_urdf_[i]->limits->lower;
                        if(error > 0)
                        {
                            old_integrator_[i] = new_integrator;
                        }
                    }
                    else
                    {
                        old_integrator_[i] = new_integrator;
                    }
                    
                    // TODO: test that
                    // js_[i] = next_pos;  // Currently not needed, could be used agin on the real robot
                    joints_[i].setCommand(next_pos);
                    old_error_[i] = error;
                    
                    d[i]  = vel_cmd - filtered_vel_[i];
                }
            }

            if(counter % 10 == 0)
            {
                if(pub_ && pub_->trylock())
                {
                    d[n_joints_] = period.toSec();
                    pub_->msg_.data = filtered_vel_;
                    pub_->unlockAndPublish();
                }
            }
            counter++; 
        }

        // void update_adaptive(const ros::Time& time, const ros::Duration& period)
        // {
        //     ROS_ERROR_STREAM("Wrong Function");
        //     std::vector<double> & commands = *commands_buffer_.readFromRT();
        //     // for(unsigned int i=0; i<n_joints_; i++)
        //     // {  
        //     //     double position = joints_[i].getPosition();
        //     //     joints_[i].setCommand(position + commands[i] * period.toSec());  
        //     // }
        //     double vel_cmd = commands[0];
        //     double vel_curr = joints_[0].getVelocity();
        //     double vel_diff = abs(vel_cmd) - abs(vel_curr);
        //     double pos_curr = joints_[0].getPosition();

        //     vel_gain = vel_gain + vel_diff * period.toSec();
        //     if(vel_cmd == 0.0)
        //     {
        //         vel_gain = 0;
        //     }

        //     double next_pos = pos_curr + vel_cmd * vel_gain * period.toSec();

        //     joints_[0].setCommand(next_pos);
        //     // ROS_ERROR_STREAM("posiiton " << joints_[0].getPosition() << " next position " << (joints_[0].getPosition() - 100 * period.toSec()));
        //     // ROS_ERROR_STREAM("Velocity of the arm flex joint " << joints_[0].getVelocity());
        //     // if(counter == 10 && pub_->trylock())
        //     // {
        //     //     std_msgs::Float64MultiArray msg;
        //     //     std::vector<double>  d(5, 0);
        //     //     d[0] = vel_cmd;
        //     //     d[1] = vel_diff;
        //     //     d[2] = vel_curr;
        //     //     d[3] = vel_gain;
        //     //     d[4] = next_pos;
        //     //     pub_->msg_.data = d;
        //     //     pub_->msg_.header.stamp = ros::Time::now();
        //     //     pub_.publish(msg);

        //     //     counter = 0;
        //     // }
        //     // counter = counter + 1; 
        // }

        std::vector< std::string > joint_names_;
        std::vector< hardware_interface::JointHandle > joints_;
        realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_;
        unsigned int n_joints_;
        unsigned int counter;
        std::vector<double> js_ ;
        std::vector<double> old_integrator_ ;
        std::vector<double> old_error_ ;
        std::vector<double> filtered_vel_ ;
        std::vector<double> p_gains_;
        std::vector<double> i_gains_;
        std::vector<double> d_gains_;
        std::vector<double> ff_gains_;
        std::vector<urdf::JointConstSharedPtr> joints_urdf_;


        private:
        ros::Subscriber sub_command_;
        std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> > pub_;

        void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
        {
            if(msg->data.size()!=n_joints_)
            {
                ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
                return;
            }
        commands_buffer_.writeFromNonRT(msg->data);
        }

    };
    PLUGINLIB_EXPORT_CLASS(hsr_velocity_controller_ns::HsrVelocityController, controller_interface::ControllerBase)
}
