#include <ros/ros.h>
#include <quadrotor_msgs/GoalSet.h>
#include <nav_msgs/Odometry.h>
#include <controller_msgs/cmd.h>

class FlightManager {
public:
    FlightManager() : nh_("~") {
        
        nh_.param<int>("drone_id", drone_id_, 0);
        nh_.param<double>("init_x", init_x_, 0.0);
        nh_.param<double>("init_y", init_y_, 0.0);
        nh_.param<double>("init_z", init_z_, 0.0);

        
        cmd_sub_ = nh_.subscribe("/control", 10, &FlightManager::cmdCallback, this);
        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &FlightManager::odomCallback, this);
        
        
        goal_pub_ = nh_.advertise<quadrotor_msgs::GoalSet>("/goal_with_id", 10);
    }

private:
    void cmdCallback(const controller_msgs::cmd::ConstPtr& msg) {
        int new_cmd_ = msg->cmd;
        if(new_cmd_ != current_cmd_) {
            previous_cmd_ = current_cmd_;
            current_cmd_ = new_cmd_;
            handleCommand();
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_pose_ = msg->pose.pose;
    }

    void handleCommand() {
        quadrotor_msgs::GoalSet goal_msg;
        goal_msg.drone_id = drone_id_;

        switch(current_cmd_) {
            case 3:  // Return
                goal_msg.goal = {
                    static_cast<float>(init_x_),
                    static_cast<float>(init_y_),
                    static_cast<float>(init_z_)
                };
                goal_pub_.publish(goal_msg);
                ROS_INFO("Return to initial position: (%.2f, %.2f, %.2f)", 
                        init_x_, init_y_, init_z_);
                break;
                
            case 4:  // Continue
                if(previous_cmd_ == 3) {
                    goal_msg.goal = {
                        static_cast<float>(current_pose_.position.x),
                        static_cast<float>(current_pose_.position.y),
                        static_cast<float>(current_pose_.position.z)
                    };
                    goal_pub_.publish(goal_msg);
                    ROS_INFO("Continue to current position: (%.2f, %.2f, %.2f)",
                            goal_msg.goal[0], goal_msg.goal[1], goal_msg.goal[2]);
                }
                break;

            default:
                break;
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher goal_pub_;
    
    int drone_id_;
    double init_x_, init_y_, init_z_;
    geometry_msgs::Pose current_pose_;
    int current_cmd_ = 0;
    int previous_cmd_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "flight_manager");
    FlightManager manager;
    ros::spin();
    return 0;
}
