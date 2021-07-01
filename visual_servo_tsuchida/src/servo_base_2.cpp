#include <visual_servo_tsuchida/servo_base.h>

void study_0::set_twist(Twi_v &velocity, vpColVector &v, vpServo &tas)
{
    v = tas.computeControlLaw();
    velocity.header.stamp = ros::Time::now();
    
    velocity.twist.linear.x = -v[0] * bai;
    velocity.twist.linear.y = v[1] * bai;
    velocity.twist.linear.z = v[2] * bai;
    velocity.twist.angular.x = -v[3] * bai;
    velocity.twist.angular.y = v[4] * bai;
    velocity.twist.angular.z = v[5] * bai;
}

bool study_0::init_setting_all(std::string camera_topic, std::string mask_image)
{
    //pro_show("fooo");
    parameter_set();
    //pro_show("pfadf");
    get_camera_info(cam_visp, camera_topic);
    //pro_show("98d");
    setworld(point);
    //pro_show("332");
    init_apriltag(detector);
    //pro_show("679");
    april_message fadfaf;
    fadfaf = init_image(Imot, Idisp, mask_image);
    mask_cdMo_detect = fadfaf.cM;
    
    //pro_show("33335");
    if (!fadfaf.detect_or_not)
    {
        return false;
    }
    set_feature_quantity(4);
    //pro_show("56334");
    visual_task_initialize(task, p, pd);
    //pro_show("fj454");
    vel_initialize(vel);
    //pro_show("35ju5");
    klt_initialize();
    //pro_show("35ccv");
    //feature_create(fadfaf.core, pd, cdMo);
    //pro_show("fj78v");
    feature_create_kai(4, pd, cdMo);
    //pro_show("fjj5)#");
}

void study_0::pose_vector_show(std::vector<double> pose)
{
    std::cout << "position x = " << pose[0] << std::endl;   
    std::cout << "position y = " << pose[1] << std::endl;   
    std::cout << "position z = " << pose[2] << std::endl;    
    std::cout << "roll = " << pose[3] << std::endl; 
    std::cout << "pitch = " << pose[4] << std::endl;
    std::cout << "yaw  = " << pose[5] << std::endl;
}

/*If you input gemetry_msgs::Pose, you can get the pose vector xyzrpy*/
std::vector<double> study_0::pose_to_vector(geometry_msgs::Pose pose)
{
    std::vector<double> pose_trans;
    geometry_msgs::Pose poseactive = pose;
    const geometry_msgs::Quaternion msgqut = poseactive.orientation;
    tf::Quaternion qual;
    double x, y, z, roll, pitch, yaw;
    tf::quaternionMsgToTF(msgqut, qual);
    tf::Matrix3x3 m(qual);
    m.getRPY(roll, pitch, yaw);
    x = poseactive.position.x;
    y = poseactive.position.y;
    z = poseactive.position.z;
    pose_trans.push_back(x);
    pose_trans.push_back(y);
    pose_trans.push_back(z);
    pose_trans.push_back(roll);
    pose_trans.push_back(pitch);
    pose_trans.push_back(yaw); 
    return pose_trans;
}

/*If you input pose vector xyzrpy, you can get geometry_msgs::Pose*/
geometry_msgs::Pose study_0::vector_to_pose(std::vector<double> xyzrpy)
{
    geometry_msgs::Pose modoriti;
    modoriti.position.x = xyzrpy[0];
    modoriti.position.y = xyzrpy[1];
    modoriti.position.z = xyzrpy[2];
    modoriti.orientation = tf::createQuaternionMsgFromRollPitchYaw(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
    return modoriti;
}

vpColVector study_0::calc_control_law(vpServo &fdfd)
{
    return fdfd.computeControlLaw();
}

geometry_msgs::Pose study_0::cMo_to_pose(Homo cMO)
{
    return visp_bridge::toGeometryMsgsPose(cMO);
}

geometry_msgs::Pose study_0::get_current_pose()
{
    return move_group->getCurrentPose().pose;
}

void study_0::feature_create(std::vector<vpImagePoint>& corners, PorPd& pdp, Homo&cmdm)
{
    for (size_t i = 0; i < corners.size(); i++) {
        vpFeatureBuilder::create(pdp[i], cam_visp, corners[i]);
        vpColVector cP;
        point[i].changeFrame(cmdm, cP);
        pdp[i].set_Z(cP[2]);
    }
}

void study_0::feature_create_kai(int kazu, PorPd& porp, Homo &cmc)
{
    for (int i = 0; i < kazu; i++) {
       // pro_show("fdfr");
        point[i].track(cmc);
        vpFeatureBuilder::create(porp[i], point[i]);
    }
}

void study_0::image_draw(Image Iout, PorPd &porp, bool green)
{
    //pro_show("display_before");
    //vpDisplay::display(Iout);
   // pro_show("display_out");
    for (int i = 0; i < porp.size(); i++) {
        std::stringstream ss;
        vpImagePoint ip;
        ss << i;
       // pro_show("pixel before");
        vpMeterPixelConversion::convertPoint(cam_visp, porp[i].get_x(), porp[i].get_y(), ip);
        //pro_show("pixel  after");
        if (green) {
            vpDisplay::displayText(Iout, ip + vpImagePoint(15, 15), ss.str(), vpColor::green);
            vpDisplay::displayCross(Iout, ip, 14, vpColor::green, 4);
        }
        else {
            vpDisplay::displayText(Iout, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
            vpDisplay::displayCross(Iout, ip, 14, vpColor::red, 4);
        }
    }
}

void study_0::image_insert(Image &Iinsert, Image &Iout)
{
    Iout.insert(Iinsert, vpImagePoint(0, 0));
}

void study_0::image_flush(Image Iout)
{
    vpDisplay::flush(Iout);
}

/*1 xyz : x = 1, y = 2, z = 3moveit_robot_model_loader
  2 sign : +1 or -1*
  3 step : each step*/
void study_0::key_move(int xyz, int sign, double step)
{
    int x_scale = 0, y_scale = 0, z_scale = 0;
    if (xyz == 1)
    {
        x_scale = 1;
    }
    else if (xyz == 2)
    {
        y_scale = 1;
    }
    else if (xyz = 3)
    {
        z_scale = 1;
    }
    else {
        ;
    }
    std::cout << "x_scale y_scale z_scale " << x_scale << " " << y_scale << " " << z_scale << std::endl;
    std::cout << "sign : " << sign << std::endl;
    std::cout << "xyz " << xyz << std::endl;
    std::cout << "step " << step << std::endl;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose wpose = move_group->getCurrentPose().pose;
    std::cout << wpose << std::endl;
    wpose.position.x += x_scale * step * sign;
    wpose.position.y += y_scale * step * sign;
    wpose.position.z += z_scale * step * sign;
    std::cout << wpose << std::endl;
    waypoints.push_back(wpose);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group->computeCartesianPath(waypoints, eef_step,
    jump_threshold, trajectory);
    //move_group->setPoseTarget(wpose);
    move_group->execute(trajectory);
    //move_group->move();
}

/*
1: save file name
2: image topic name*/
void study_0::mask_image_save(std::string image_file, std::string image_topic_1)
{
    sensor_msgs::ImageConstPtr share;
    share = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_1, ros::Duration(10));
    cv_bridge::CvImagePtr input_bridge;
    if (share != NULL) {
        input_bridge = cv_bridge::toCvCopy(share, sensor_msgs::image_encodings::BGR8);
    }
    cv::Mat img;
    img = input_bridge->image;
    cv::imshow("mask show", img);
    cv::waitKey(3000);
    cv::imwrite(image_file, img);
    cv::destroyAllWindows();
    share.reset();
}

/*get current joint value and set to start_joint_position*/
void study_0::get_joint_value(std::vector<double> &start_joint)
{
    const robot_state::JointModelGroup* joint_model_group =
      move_group->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    moveit::core::RobotStatePtr current_state = move_group->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, start_joint);
}

void study_0::home_position_return()
{
    move_group->setJointValueTarget(start_joint_position);
    move_group->move();
}

double study_0::average_error(vpColVector error)
{
    double ave = 0;
    int count = 0;
    double y4;
    for (int i = 0; i < error.size(); i++) {
        if (error[i] > 0) {
            y4 = error[i];
        } else {
            y4 = -error[i];
        }
        ave = ave + y4;
        count++;
    }
    ave = ave / count;
    return ave;
}

void study_0::parameter_set()
{
    pnh->getParam("lamda", lamda_num);
    bai = 3;
    pnh->getParam("bai", bai);
    pnh->getParam("tagSize", tag_size);
    pnh->getParam("feature_quantity", feature_quantity);
    pnh->getParam("mask_file", mask_file_name);
    pnh->getParam("end_pose_file", end_pose_file_name);
    pnh->getParam("error_pose_file", error_pose_file_name);
}

void study_0::mask_pose_vector_set()
{
    mask_pose_vector = pose_to_vector(get_current_pose());
    std::cout << "mask pose is ";
    for (int i = 0; i < mask_pose_vector.size(); i++) {
        std::cout << mask_pose_vector[i] << " ";
    }
    std::cout << std::endl;
    write_value_in_file(mask_out, mask_file_name, mask_pose_vector);
}

void study_0::end_pose_vector_set()
{
    outcome_pose_vector = pose_to_vector(get_current_pose());
    std::cout << "end pose is ";
    for (int i = 0; i < outcome_pose_vector.size(); i++) {
        std::cout << outcome_pose_vector[i] << " ";
    }
    std::cout << std::endl;
    write_value_in_file(end_pose_out, end_pose_file_name, outcome_pose_vector);
}

void study_0::error_pose_calcurate()
{
    if (mask_pose_vector.size() != outcome_pose_vector.size()) {
        ROS_ERROR_STREAM("Not mathc size of vector");
    } else {
        std::cout << "presision is ";
        std::vector<double> itiji;
        for (int i = 0; i < mask_pose_vector.size(); i++) {
            itiji.push_back(mask_pose_vector[i] - outcome_pose_vector[i]);
            std::cout << itiji[i] << " ";
        }
        std::cout << std::endl;
        error_pose_vector = itiji;
       // pro_show("error file name is");
        //pro_show(error_pose_file_name);
        write_value_in_file_overwrite(error_out, error_pose_file_name, error_pose_vector);
    }
}

std::vector<std::string> study_0::split(std::string &input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}


