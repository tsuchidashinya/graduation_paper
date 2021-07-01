#pragma once
#include "servo_base.h"
#include <actionlib/server/simple_action_server.h>
#include <visual_servo_tsuchida/Servo_messageAction.h>

typedef actionlib::SimpleActionServer<visual_servo_tsuchida::Servo_messageAction> Visu_ser;

class Servo_actionserver
{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle *pnh_;
    Visu_ser server;
    std::string name_action;
    study_0 *study;
    ros::Subscriber sub;
    visual_servo_tsuchida::Servo_messageFeedback feedback;
    ros::Publisher twist_pub;
    std::string camera_info_topic = "camera/camera_info";
    std::string jog_topic = "/jog_server/delta_jog_cmds";
    std::string mask_image_file = "/home/tsuchida/image/mask.jpg";
    std::string image_topic_name = "camera/image_raw";
    std::string moveit_group;
    double error = 0.01;
    double linear_error = 0.01;
    double angular_error = 0.05;
    pose_message Threshould_error;
    double loop_rate = 14;
    visual_servo_tsuchida::Servo_messageResult result_;
    clock_t start, end;
   
public:
    Servo_actionserver(std::string name): moveit_group("manipulator"), server(nh_, name, boost::bind(&Servo_actionserver::exec_action_callback, this, _1), false)
    {
        pnh_ = new ros::NodeHandle("~");
        parameter_set();
        study = new study_0(moveit_group);   
        server.start();
        ros::AsyncSpinner spinner(1);
        spinner.start();
        
        
    }
    void exec_action_callback(const visual_servo_tsuchida::Servo_messageGoalConstPtr&);
    void visual_servo();
    
    void image_callback(const sensor_msgs::ImageConstPtr&, study_0*);
    void kawari_image(sensor_msgs::ImageConstPtr, study_0*);
    void callback(const visual_servo_tsuchida::Servo_messageGoalConstPtr &msg);
    void parameter_set();
    bool evaluation_pose_error(pose_message, pose_message);
};