#include <visual_servo_tsuchida/client_visual.h>

Servo_client::Servo_client(std::string server_name)
{
    pnh = new ros::NodeHandle("~");
    parameter_set();
    action_client = new Client(server_name);
    study = new study_0(moveit_group);
    ROS_INFO_STREAM("Waiting for server");
    action_client->waitForServer();
    ROS_INFO_STREAM("Visual Servo Start");
    
}

void Servo_client::visual_send_goal(pose_message error_all)
{
    visual_servo_tsuchida::Servo_messageGoal goal;
    goal.goal_linear_error = error_all.linear_error;
    goal.goal_angular_error = error_all.anglar_error;
    goal.option = 1;
    action_client->sendGoal(goal);
}

void Servo_client::visual_send_goal(double eoor)
{
    visual_servo_tsuchida::Servo_messageGoal gaol;
    gaol.goal_error = eoor;
    gaol.option = 1;
    action_client->sendGoal(gaol);
}

bool Servo_client::minimum_move(double step)
{
    int xyz;
    int sign;
    std::string key;
    bool yameru = false;
    text_show("x++ : a   x-- : d ");
    text_show("y++ : i   y-- : m ");
    text_show("z++ : e   z-- : x ");
    std::cin >> key;
    if (key == "a")
    {
        xyz = 1;
        sign = 1;
    } else if (key == "d")
    {
        xyz = 1;
        sign = -1;
    } else if (key == "i")
    {
        xyz = 2;
        sign = 1;
    } else if (key == "m")
    {
        xyz = 2;
        sign = -1;
    } else if (key == "e")
    {
        xyz = 3;
        sign = 1;
    } else if (key == "x")
    {
        xyz = 3;
        sign = -1;
    } else {
        xyz = 0;
        sign = 0;
        yameru = true;
        /*do {
            text_show("Do you quit? (y/n)");
            std::string ym;
            std::cin >> ym;
            if (ym == "y")
            {
                yameru = true;
                break;
            }
            else if (ym == "n")
            {
                break;
            }
            else {
                ;
            }
        } while (true);*/
        
    }
    study->key_move(xyz, sign, step);
    return yameru;
}

void Servo_client::text_show(std::string str)
{
    std::cout << str << std::endl;
}

void Servo_client::send_halt()
{
    action_client->cancelGoal();
}

void Servo_client::client_image_save()
{
    study->mask_image_save(mask_image_file, image_topic);
    visual_servo_tsuchida::Servo_messageGoal goal;
    goal.option = 2;
    action_client->sendGoal(goal);

}

void Servo_client::register_position()
{
    study->get_joint_value(study->start_joint_position);
    std::cout << "home position register" << std::endl;
}

void Servo_client::return_position()
{
    study->home_position_return();
}

void Servo_client::parameter_set()
{
    pnh->getParam("image_topic_name_client", image_topic);
    pnh->getParam("mask_image_client", mask_image_file);
    pnh->getParam("moveit_client", moveit_group);
}

/*
If you got result, show true
*/
bool Servo_client::result_suceess_or_not()
{
    return action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
}