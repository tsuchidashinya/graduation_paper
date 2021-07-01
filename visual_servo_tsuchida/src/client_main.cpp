#include <visual_servo_tsuchida/client_visual.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ini");

    //ros::AsyncSpinner spinner(1);

   // spinner.start();

    ros::AsyncSpinner spinner(1);
    spinner.start();

  //spinner.start();
    double minumam_step = 0.03;
    double error = 0.01;
    double linear_error = 0.01;
    double angular_error = 0.05;
    ros::NodeHandle pnh("~");
    std::string client_server_name;
    pnh.getParam("client_server_name", client_server_name);
    pnh.getParam("moveit_step", minumam_step);
    pnh.getParam("linear_error", linear_error);
    pnh.getParam("angular_error", angular_error);
    pnh.getParam("error", error);
    pose_message pose_error;
    pose_error.linear_error = linear_error;
    pose_error.anglar_error = angular_error;
    Servo_client client(client_server_name);
    do {
        client.text_show("1: visual servo");
        client.text_show("2: move");
        client.text_show("3: preempted");
        client.text_show("4: capture");
        client.text_show("5: register home position");
        client.text_show("6: return home position");
        client.text_show("q: quit");
        std::string key;
        std::cin >> key;
        bool move_yameru = false;
        if (key == "1")
        {
            /*std::cout << "How much error value?" << std::endl;
            double er;
            std::cin >> er;*/
            client.visual_send_goal(error);
        }
        else if(key == "2")
        {
            while (!client.minimum_move(minumam_step)) {
            }
        } else if (key == "3"){
            client.send_halt();
        } else if (key == "4")
        {
            client.client_image_save();
        } else if (key == "5")
        {
            client.register_position();
        } else if (key == "6") {
            client.return_position();
        }

        else {
          break;
        }

    } while (true);
    return 0;

}
