#include <visual_servo_tsuchida/servo_base.h>

typedef moveit::planning_interface::MoveGroupInterface MoveInter;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dfa");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    study_0 stu("manipulator");
    ros::Rate r(1);
    while (ros::ok())
    {
        std::vector<double> pose;
        pose = stu.pose_to_vector(stu.get_current_pose());
        std::cout << "pose is ";
        for (int i = 0; i < pose.size(); i++) {
            std::cout << pose[i] << " ";
        }
        std::cout << std::endl;
        r.sleep();
    }
    spinner.stop();
    return 0;

}