#pragma once
#include "servo_base.h"
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <visual_servo_tsuchida/Servo_messageAction.h>


typedef actionlib::SimpleActionClient<visual_servo_tsuchida::Servo_messageAction> Client;

class Servo_client
{
protected:
    Client *action_client;
    study_0 *study;
    ros::NodeHandle nh;
    ros::NodeHandle *pnh;
    std::string mask_image_file;
    std::string image_topic;
    std::string moveit_group;

public:
    Servo_client(std::string);
    void visual_send_goal(pose_message);
    void visual_send_goal(double);
    bool minimum_move(double);
    void text_show(std::string);
    void send_halt();
    void client_image_save();
    void register_position();
    void return_position();
    void parameter_set();
    bool result_suceess_or_not();
    
};
