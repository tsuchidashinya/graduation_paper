#include <visual_servo_tsuchida/servo_base.h>


study_0::study_0(std::string group_name): tagFamily(vpDetectorAprilTag::TAG_36h11), tag_size(0.05), lamda_num(-0.5),
                pose_method(vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS), feature_quantity(4), message_kazu(1)
{
    pnh = new ros::NodeHandle("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    pnh->getParam("moveit_group", PLANNING_GROUP);
    parameter_set();
    detector = new vpDetectorAprilTag(tagFamily);
    move_group = new MoveInter(group_name);
    vpFeaturePoint fdf;
    double yd = 0;
    double xd = 0;
    double Zd = 1;
    fdf.buildFrom(xd, yd, Zd);
    for (int i = 0; i < feature_quantity; i++) {
        p.push_back(fdf);
        pd.push_back(fdf);
    }
    get_joint_value(start_joint_position);
    //pro_show("mask_file_name is");
    //pro_show(mask_file_name);
    mask_pose_vector = input_value(mask_in, mask_file_name);
    outcome_pose_vector = input_value(outcome_in, end_pose_file_name);
    std::cout << "mask pose vector is ";
    vector_show(mask_pose_vector);
    std::cout << "outcome pose vector is";
    vector_show(outcome_pose_vector);
    
}

void study_0::Image_point_init(std::vector<vpImagePoint>& isl)
{
    double ef = 23;
    vpImagePoint haji(ef, ef);
    isl.push_back(haji);
    isl.push_back(haji);
    isl.push_back(haji);
    isl.push_back(haji);
}

april_message study_0::init_image(Image &Imotmot, Image &Idispdisp, std::string mask_image_1)
{
    vpImageIo::read(Imotmot, mask_image_1);
    Idispdisp.resize(Imotmot.getHeight(), Imotmot.getWidth());
    Idispdisp.insert(Imotmot, vpImagePoint(0, 0));
    vpDisplayX d(Idispdisp, 0, 0, "");
    vpDisplay::display(Idispdisp);
    vpDisplay::flush(Idispdisp);
    april_message out;
    out = april_detect(Imotmot);
    april_interface = out;
    bool ikeru;
    if (out.detect_or_not)
    {
        cdMo = out.cM;
        pro_show("init image clear");
        ikeru = true;
    }
    else {
        pro_show("init image false");
        ikeru = false;
    }
    return out;
}

void study_0::klt_initialize()
{
    klt_settings.setMaxFeatures(300);
    klt_settings.setWindowSize(5);
    klt_settings.setQuality(0.015);
    klt_settings.setMinDistance(8);
    klt_settings.setHarrisFreeParameter(0.01);
    klt_settings.setBlockSize(3);
    klt_settings.setPyramidLevels(3);
}

void study_0::tracker_initialize()
{
}

void study_0::visual_task_initialize(vpServo &task_1, PorPd &pcurrent, PorPd &pdestination)
{
    task_1.setServo(vpServo::EYEINHAND_CAMERA);
    task_1.setInteractionMatrixType(vpServo::CURRENT);
    task_1.setLambda(lamda_num);
    for (int i = 0; i < feature_quantity; i++) {
        task_1.addFeature(pcurrent[i], pdestination[i]);
    }
}

void study_0::init_apriltag(vpDetectorAprilTag* det)
{
    det->setAprilTagPoseEstimationMethod(pose_method);
    det->setAprilTagQuadDecimate(opt_quad_decimate);
}


void study_0::get_camera_info(vpCameraParameters &camm, std::string camera_topic_name)
{
    sensor_msgs::CameraInfoConstPtr share;
    share = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_topic_name, ros::Duration(10));
    sensor_msgs::CameraInfo cp;
    if (share != NULL) {
        cp = *share;
    }
    camm = visp_bridge::toVispCameraParameters(*share);
}

void study_0::vel_initialize(vpColVector &vel_1)
{
    vel_1.resize(6);
}

april_message study_0::april_detect(const sensor_msgs::ImageConstPtr& msg, vpCameraParameters ca)
{
    Iin = visp_bridge::toVispImage(*msg);
    april_message output;
    output.detect_or_not = detector->detect(Iin, tag_size, ca, cdMo_vec);
    output.cM = cdMo_vec[cdMo_vec.size() - 1];
    return output;
}

april_message study_0::april_detect(const sensor_msgs::ImageConstPtr& msg, Image& If)
{
    If = visp_bridge::toVispImage(*msg);
    april_message output;
    VecHomo kensa;
    Homo kawari(0, 0.0, 0.75, 0, 0, 0);
    output.detect_or_not = detector->detect(If, tag_size, cam_visp, kensa);
    if (output.detect_or_not)
    {
        output.cM = kensa[kensa.size() - 1];
        output.core = detector->getPolygon(0);
    }
    else {
        ROS_ERROR_STREAM("AR is not detected");
        output.cM = kawari;
        int size = kensa.size();
    }
    return output;
}

april_message study_0::april_detect(const sensor_msgs::ImageConstPtr& msg, Image& If, Homo &cMocMo, std::vector<vpImagePoint>& Ippo)
{
    If = visp_bridge::toVispImage(*msg);
    april_message output;
    VecHomo kensa;
    output.detect_or_not = detector->detect(If, tag_size, cam_visp, kensa);
    if (output.detect_or_not)
    {
        output.cM = kensa[kensa.size() - 1];
        output.core = detector->getPolygon(0);
    }
    else {
        output.cM = cMocMo;
        int size = kensa.size();
        output.core = Ippo;
    }
    return output;
}

april_message study_0::april_detect(Image& im)
{
    april_message output;
    VecHomo kensa;
    output.detect_or_not = detector->detect(im, tag_size, cam_visp, kensa);
    output.core = detector->getPolygon(0);
    int size = kensa.size();
    if (output.detect_or_not)
    {
        output.cM = kensa[size - 1];
        std::cout << "detect success" << std::endl;
    }
    else {
        Homo cfd(0, 0, 0.56, 0, 0, 0);
        output.cM = cfd;
        std::cout << "detect fail" << std::endl;
    }
    return output;
}

void study_0::april_detect_1(const sensor_msgs::ImageConstPtr& msg, vpCameraParameters ca)
{
    Iin = visp_bridge::toVispImage(*msg);
    april_message output;
    VecHomo kensa;
    output.detect_or_not = true;
    output.detect_or_not = detector->detect(Iin, tag_size, ca, kensa);
    output.core = detector->getPolygon(0);
    if (output.detect_or_not)
        std::cout << "true" << std::endl;
    else {
        std::cout << "false" << std::endl;
    }
}
void study_0::set_feature_quantity(int num)
{
    feature_quantity = num;
}

void study_0::setworld(VPoint &pon)
{
    vpPoint ponpon;
    ponpon.setWorldCoordinates(-tag_size/2, -tag_size/2, 0);
    pon.push_back(ponpon);
    ponpon.setWorldCoordinates(tag_size/2, -tag_size/2, 0);
    pon.push_back(ponpon);
    ponpon.setWorldCoordinates(tag_size/2, tag_size/2, 0);
    pon.push_back(ponpon);
    ponpon.setWorldCoordinates(-tag_size/2, tag_size/2, 0);
    pon.push_back(ponpon);
    //pro_show("point[0]");
    show_various(pon[0].getWorldCoordinates());
    //pro_show("point[1]");
    show_various(pon[1]);
}

void study_0::pro_show(std::string str)
{
    std::cout << message_kazu++ << " : " <<  str << std::endl;
}
