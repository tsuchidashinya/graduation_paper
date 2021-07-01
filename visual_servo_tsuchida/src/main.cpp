
/*#include <visualization_msgs/Marker.h>
#include <visual_servo_tsuchida/servo_base.h>
#include <math.h>*/
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>

std::vector<std::string> split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

std::vector<std::string> file_input(std::ifstream *in, std::string filename)
{
    in = new std::ifstream(filename);
    std::string line;   
    std::getline(*in, line);
    std::vector<std::string> strvec = split(line, ' ');
    return strvec;
}

std::vector<double> string_to_double(std::vector<std::string> moji)
{
    std::vector<double> resu;
    for (int i = 0; i < moji.size(); i++) {
        resu.push_back(std::stod(moji[i]));
    }
    return resu;
}

void write_value_in_file(std::ofstream* ofs, std::string name, std::vector<double> value)
{
    ofs = new std::ofstream(name);
    for (int i = 0; i < value.size(); i++) {
        *ofs << value[i] << " ";
    }
    *ofs << std::endl;
}

void write_value_in_file_overwrite(std::ofstream *ofs, std::string name, std::vector<double> value)
{
    //ofs = new std::ofstream(name, std::ios::in | std::ios::ate);
    ofs = new std::ofstream(name, std::ios::app);
    for (int i = 0; i < value.size(); i++) {
        *ofs << value[i] << " ";
    }
    *ofs << std::endl;
}

std::vector<double> input_value(std::ifstream *ifs, std::string name)
{
    return string_to_double(file_input(ifs, name));
}

/*std::ofstream *t = new std::ofstream("/home/tsuchida/cMO.txt");
void callback(const sensor_msgs::ImageConstPtr& msg, study_0 &stdy, vpCameraParameters &c)
{

    april_message ten = stdy.april_detect(msg, c);
    //stdy.april_detect_1(msg, c);
    if (ten.detect_or_not)
    {
        geometry_msgs::Pose s = visp_bridge::toGeometryMsgsPose(ten.cM);
        std::cout << "cMo is " << std::endl;
        stdy.pose_vector_show(stdy.pose_to_vector(s));
    }  
}
void ar_callback(const visualization_msgs::MarkerConstPtr& msg, study_0 &ke)
{
    
    /*stdy.pose_show(msg->pose, t);
    *t << std::endl;
    *t << std::endl;*/
   /* ROS_INFO_STREAM(msg->pose);
    //std::cout << msg->pose << std::endl;
    ke.pose_vector_show(ke.pose_to_vector(msg->pose));
    ROS_INFO_STREAM("had");
    
}*/
int main(int argc, char** argv)
{
    std::string filename = "/home/tsuchida/study_result/mask_file.txt";
    std::ifstream *ifs;
    //std::ofstream ofs("/home/tsuchida/file_process/outwe.txt", std::ios::in | std::ios::ate);
    std::ofstream *ofs;
    std::vector<double> relu;
    relu = input_value(ifs, filename);
    write_value_in_file_overwrite(ofs, filename, relu);
    /*ofs = new std::ofstream(filename);
    for (int i = 0; i < relu.size(); i++) {
        std::cout << relu[i] << " ";
        *ofs << relu[i] + 1 << " ";  
    }
    std::cout << std::endl;
    *ofs << std::endl;
    return 0;
    //ros::init(argc, argv, "init");
    /*std::vector<double> haji;
    for (int i = 0; i < 3; i++) {
        haji.push_back(0);
    }
    haji.push_back(M_PI);
    haji.push_back(0);
    haji.push_back(M_PI / 2);
    geometry_msgs::Pose ls;
    ls = study_0::vector_to_pose(haji);
    std::cout << ls << std::endl;
    haji = study_0::pose_to_vector(ls);
    for (int i = 0; i < 6; i++) {
        std::cout << haji[i] << " ";
    }

    
    std::cout  << std::endl;*/

    
    //study_0 stu("manipulator");
    /*stu.init_setting_all("usb_cam_camera_info", "/home/tsuchida/image/mask.jpg");
    vpCameraParameters cma;
    stu.get_camera_info(cma, "/usb_cam/camera_info");
    std::cout << "hi" << std::endl;
    ros::Subscriber ar_sub, image_sub;
    ros::NodeHandle nh;
    std::cout << "yes" << std::endl;

    //ar_sub = nh.subscribe<visualization_msgs::Marker>("/visualization_marker", 1000, boost::bind(ar_callback, _1, stu));
    image_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 1000, boost::bind(callback, _1, stu, cma));
    ros::Rate loop(1);
    std::cout << "ninh" << std::endl;
    while (ros::ok())
    {
        std::cout << "ama" << std::endl;
        ROS_INFO_STREAM("mada nanimo");
        ros::spinOnce();
        loop.sleep();
    }
    //ros::spin();
    return 0;*/
}