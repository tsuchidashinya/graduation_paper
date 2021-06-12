#pragma once
#include <visp_bridge/camera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp_bridge/image.h>
#include <visp_bridge/3dpose.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <ros/ros.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint3D.h>
#include <visp3/vs/vpServo.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/camera.h>
#include <cv_bridge/cv_bridge.h>
#include <visp3/vs/vpServo.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpVideoReader.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpImageDraw.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <fstream>
#include <cmath>
#include <limits>
#include <tf/tf.h>
#include <visp_bridge/image.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

typedef moveit::planning_interface::MoveGroupInterface MoveInter;
typedef geometry_msgs::TwistStamped Twi_v;
typedef vpImage<unsigned char> Image;
typedef std::vector<vpFeaturePoint> PorPd;
typedef std::vector<vpPoint> VPoint;
typedef vpHomogeneousMatrix Homo;
typedef std::vector<Homo> VecHomo;

struct april_message {
    bool detect_or_not;
    Homo cM;
    std::vector<vpImagePoint> core;
};

struct pose_message {
    double linear_error;
    double anglar_error;
};

class study_0 {
protected:
    std::string PLANNING_GROUP = "manipulator";
    std::string mask_file_name;
    std::string end_pose_file_name;
    std::string error_pose_file_name;
    ros::NodeHandle *pnh;
    double lamda_num;
    double tag_size;
    int opt_quad_decimate;
    vpKeyPoint::vpFilterMatchingType filterType;
    VecHomo cMo_vec, cdMo_vec;
    Homo cMo, cdMo;
    vpDetectorAprilTag *detector, *mask_detect;
    vpDetectorAprilTag::vpPoseEstimationMethod pose_method;
    MoveInter *move_group;
    VPoint point;
    int feature_quantity;
    int message_kazu;
    vpDetectorAprilTag::vpAprilTagFamily tagFamily;
    vpKltOpencv klt_settings;
    vpColVector vel;
    std::string sl;
    std::ofstream *mask_out, *end_pose_out, *error_out;
    std::ifstream *mask_in, *outcome_in, *error_in;

    double bai;
    std::vector<double> outcome_pose_vector;
    std::vector<double> mask_pose_vector;
    

public:
    study_0(std::string);
    std::vector<double> error_pose_vector;
    std::vector<double> start_joint_position;
    Image Iin, Imot, Idisp;
    vpServo task;
    vpCameraParameters cam_visp;
    PorPd p, pd;
    Homo mask_cdMo_detect, cMo_detect;
    void klt_initialize();
    void set_feature_quantity(int);
    april_message april_interface;
    april_message init_image(Image&, Image&, std::string);
    void init_apriltag(vpDetectorAprilTag*);
    void tracker_initialize();
    void visual_task_initialize(vpServo&, PorPd&, PorPd&);
    void get_camera_info(vpCameraParameters &cam, std::string cam_Topic);
    void vel_initialize(vpColVector&);
    void Image_point_init(std::vector<vpImagePoint>&);

    /*1. camera info topic input
      2. mask image path*/
    bool init_setting_all(std::string, std::string);
    void pose_vector_show(std::vector<double> pose);

    april_message april_detect(const sensor_msgs::ImageConstPtr&, Image&);
    april_message april_detect(const sensor_msgs::ImageConstPtr&, Image&, Homo&, std::vector<vpImagePoint>&);
    april_message april_detect(const sensor_msgs::ImageConstPtr&, vpCameraParameters);
    april_message april_detect(Image&);
    void april_detect_1(const sensor_msgs::ImageConstPtr&, vpCameraParameters);

    static std::vector<double> pose_to_vector(geometry_msgs::Pose pose);
    geometry_msgs::Pose cMo_to_pose(Homo cMO);
    geometry_msgs::Pose get_current_pose();
    void setworld(VPoint&);

    /*1. feature quantity*/
    void feature_create(std::vector<vpImagePoint>&, PorPd&, Homo&);
    void feature_create_kai(int, PorPd&, Homo&);
    void set_twist(Twi_v&, vpColVector&, vpServo&);
    void pro_show(std::string);
    vpColVector calc_control_law(vpServo &);
    void image_draw(Image, PorPd&, bool);
    void image_insert(Image&, Image&);
    void image_flush(Image);
    template <typename T>
    void show_various(T);
    void key_move(int, int, double);
    void mask_image_save(std::string, std::string);
    void get_joint_value(std::vector<double>&);
    void home_position_return();
    double average_error(vpColVector);
    static geometry_msgs::Pose vector_to_pose(std::vector<double>);
    void parameter_set();
    void mask_pose_vector_set();
    void end_pose_vector_set();
    void error_pose_calcurate();
    std::vector<std::string> split(std::string&, char);
    std::vector<double> string_to_double(std::vector<std::string>);
    std::vector<std::string> file_input(std::ifstream*, std::string);
    void write_value_in_file(std::ofstream*, std::string, std::vector<double>);
    void write_value_in_file_overwrite(std::ofstream*, std::string, std::vector<double>&);
    void vector_show(std::vector<double>);
    std::vector<double> input_value(std::ifstream*, std::string);
    pose_message max_Homo_error(Homo, Homo);

};

template <typename T>
void study_0::show_various(T tr)
{
    std::cout << tr << std::endl;
}
