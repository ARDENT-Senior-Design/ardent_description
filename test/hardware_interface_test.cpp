#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include "std_msgs/Float64.h"
class MyRobot : public hardware_interface::RobotHW
{
    public:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        hardware_interface::EffortJointInterface jnt_effort_interface;
        double cmd[2];
        double pos[2];
        double vel[2];
        double eff[2];

        ros::NodeHandle nh;

        MyRobot() 
        { 
            std::string joint_name_1;
            std::string joint_name_2;
            nh.getParam("/j_femur_1_position_controller/joint", joint_name_1);
            ROS_INFO("%s was added to the controller list", joint_name_1.c_str());
            nh.getParam("/j_femur_1_position_controller/joint", joint_name_2);
            ROS_INFO("%s was added to the controller list", joint_name_2.c_str());
            
            // connect and register the joint state interface
            hardware_interface::JointStateHandle state_handle_a(joint_name_1, &pos[0], &vel[0], &eff[0]);
            jnt_state_interface.registerHandle(state_handle_a);

            hardware_interface::JointStateHandle state_handle_b(joint_name_2, &pos[1], &vel[1], &eff[1]);
            jnt_state_interface.registerHandle(state_handle_b);

            registerInterface(&jnt_state_interface);

            // connect and register the joint position interface
            hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle(joint_name_1), &cmd[0]);
            jnt_pos_interface.registerHandle(pos_handle_a);

            hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle(joint_name_2), &cmd[1]);
            jnt_pos_interface.registerHandle(pos_handle_b);

            // hardware_interface::JointHandle vel_handle_a(jnt_state_interface.getHandle(joint_name_1), &cmd[0]);
            // jnt_vel_interface.registerHandle(vel_handle_a);

            // hardware_interface::JointHandle vel_handle_b(jnt_state_interface.getHandle(joint_name_2), &cmd[1]);
            // jnt_vel_interface.registerHandle(vel_handle_b);

            // hardware_interface::JointHandle effort_handle_a(jnt_state_interface.getHandle(joint_name_1), &cmd[0]);
            // jnt_effort_interface.registerHandle(effort_handle_a);

            // hardware_interface::JointHandle effort_handle_b(jnt_state_interface.getHandle(joint_name_2), &cmd[1]);
            // jnt_effort_interface.registerHandle(effort_handle_b);

            registerInterface(&jnt_pos_interface);
            // registerInterface(&jnt_vel_interface);
            // registerInterface(&jnt_effort_interface);
        }
        // void read(const ros::Time& time, const ros::Duration& 	period )
        // {

        // }
        // void write(const ros::Time& time, const ros::Duration& 	period )
        // {
        //     // femur_pub = nh.advertise<std_msgs::Float64>("/j_femur_1_position_controller/command",1000);
        //     // tibia_pub = nh.advertise<std_msgs::Float64>("/j_tibia_1_position_controller/command",1000);
        //     // std_msgs::Float64 femur_msg;
        //     // std_msgs::Float64 tibia_msg;
        //     // femur_msg.data = cmd[0];
        //     // tibia_msg.data = cmd[1];

        //     // femur_pub.publish(femur_msg);
        //     // tibia_pub.publish(tibia_msg);
        // }

    
    private:
    
};

float pose_cmd;

void timerCallback(const ros::TimerEvent& event){
   pose_cmd *= -1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joint_test");
    ros::NodeHandle n;
    pose_cmd =1;
    MyRobot robot;
    controller_manager::ControllerManager cm(&robot);
    ros::Timer timer = n.createTimer(ros::Duration(2.0), timerCallback);
    ros::Rate r(10);    // same as duration
    while(true)
    {

        ROS_INFO("Joint A Positions: %f", (float)robot.pos[0]);
        ROS_INFO("Joint B Positions: %f", (float)robot.pos[1]);
        ROS_INFO("Joint A Command: %f", (float)robot.cmd[0]);
        ROS_INFO("Joint B Command: %f", (float)robot.cmd[1]);
        // robot.read();
        robot.read(ros::Time::now(),ros::Duration(0.1)); // only needed if physical
        cm.update(ros::Time::now(),ros::Duration(0.1));
        robot.cmd[0] = pose_cmd;
        robot.cmd[1] = -pose_cmd;
        robot.write(ros::Time::now(),ros::Duration(0.1));
        // robot.write();
        r.sleep();
        ros::spin();
    }

}