#include <visual_servo_tsuchida/server_visual.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "faf");
    //ros::AsyncSpinner spinner(1);
   // spinner.start();
    ros::NodeHandle pnh("~");
    std::string server_name;
    pnh.getParam("server_name", server_name);
    Servo_actionserver srer(server_name);
    ros::spin();
    return 0;    
}
