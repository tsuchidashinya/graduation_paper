#include <visual_servo_tsuchida/server_visual.h>
#include <visual_servo_tsuchida/servo_base.h>
#include <cv_bridge/cv_bridge.h>
#include <time.h>


Image Idisp, Imot, Iin;
std::vector<vpImagePoint> haji;
static bool shori = true;
Homo kawari(0, 0.0, 0.75, 0, 0, 0);
void Servo_actionserver::exec_action_callback(const visual_servo_tsuchida::Servo_messageGoalConstPtr &msg)
{
    if (msg->option == 1)
    {
        //std::cout << "fdafd" << std::endl;
        error = msg->goal_error;
        Threshould_error.linear_error = msg->goal_linear_error;
        Threshould_error.anglar_error = msg->goal_angular_error;
        visual_servo();
    }
    else if (msg->option == 2)
    {
        study->mask_pose_vector_set();
        server.setSucceeded();
    }
    else {
        ;
    }
}

void Servo_actionserver::visual_servo()
{

    start = clock();
    study->init_setting_all(camera_info_topic, mask_image_file);
    study->Image_point_init(haji);
    vpImageIo::read(Imot, mask_image_file);
    Idisp.resize(Imot.getHeight(), Imot.getWidth());
    Idisp.insert(Imot, vpImagePoint(0, 0));
    vpDisplayOpenCV d(Idisp, 0, 0, "Matiching keypoits");
    vpDisplay::display(Idisp);
    vpDisplay::flush(Idisp);
    //sub = nh_.subscribe<sensor_msgs::Image>("/image_raw", 1000, boost::bind(&Servo_actionserver::image_callback, this, _1, study));
    twist_pub = nh_.advertise<geometry_msgs::TwistStamped>(jog_topic, 1000);
    ros::Rate loop(14);

    double ror;
    pose_message hyouka;
    sensor_msgs::ImageConstPtr image;
    while (ros::ok())
    {
        if (server.isPreemptRequested()) {
            server.setPreempted();
            break;
        }
        //end_1 = clock();
        //std::cout << "loop time = " << (double)(end_1 - start_1) / CLOCKS_PER_SEC << "sec.\n";
        //start_1 = clock();
        image = ros::topic::waitForMessage<sensor_msgs::Image>(image_topic_name, ros::Duration(10));
        kawari_image(image, study);
        ror = study->average_error(study->task.error);
        hyouka = study->max_Homo_error(study->cMo_detect, study->mask_cdMo_detect);
        //if (evaluation_pose_error(Threshould_error, hyouka)) {
        if (ror <= error) {
            study->end_pose_vector_set();
            study->error_pose_calcurate();
            for (int i = 0; i < study->error_pose_vector.size(); i++) {
                result_.error_result.push_back((float)study->error_pose_vector[i]);
            }
            result_.final_pose = study->get_current_pose();
            end = clock();
            result_.time_take = (double)(end - start) / CLOCKS_PER_SEC;
            std::cout << "time is " << result_.time_take << " sec" << std::endl;
            server.setSucceeded(result_);
            break;
        }
        //std::cout << ror << std::endl;

        //ros::spinOnce();
        loop.sleep();

    }
}

void Servo_actionserver::image_callback(const sensor_msgs::ImageConstPtr &image_msg, study_0 *study_class)
{
    clock_t start = clock();
    shori = false;
    Iin = visp_bridge::toVispImage(*image_msg);
    Idisp.insert(Iin, vpImagePoint(0, 0));
    april_message hantei;
    hantei = study_class->april_detect(image_msg, study_class->Iin, kawari, haji);
    kawari = hantei.cM;
    haji = hantei.core;
    Homo cmkd = hantei.cM;
    //study_class->feature_create(hantei.core, study_class->p, hantei.cM);
    study_class->feature_create_kai(4, study_class->p, hantei.cM);
    vpDisplay::display(Idisp);
    PorPd pdf;
    pdf = study_class->pd;
    for (int i = 0; i < pdf.size(); i++) {
        std::stringstream ss;
        vpImagePoint ip;
        ss << i;
        vpMeterPixelConversion::convertPoint(study_class->cam_visp, pdf[i].get_x(), pdf[i].get_y(), ip);
        vpDisplay::displayText(Idisp, ip + vpImagePoint(15, 15), ss.str(), vpColor::green);
        vpDisplay::displayCross(Idisp, ip, 14, vpColor::green, 4);
    }
    pdf = study_class->p;
    for (int i = 0; i < pdf.size(); i++) {
        std::stringstream ss;
        vpImagePoint ip;
        ss << i;
        vpMeterPixelConversion::convertPoint(study_class->cam_visp, pdf[i].get_x(), pdf[i].get_y(), ip);
        vpDisplay::displayText(Idisp, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
        vpDisplay::displayCross(Idisp, ip, 14, vpColor::red, 4);
    }
    vpColVector vere = study_class->calc_control_law(study_class->task);
    /*for (int i = 0; i < 6; i++) {
        std::cout << study_class->task.computeControlLaw()[i] << " ";
    }
    std::cout << std::endl;*/
    geometry_msgs::TwistStamped hasiru;
    study_class->set_twist(hasiru, vere, study_class->task);
    double ror;
    ror = study_class->average_error(study->task.error);
    if (ror <= error) {
        server.setSucceeded();
        hasiru.twist.angular.x = 0;
        hasiru.twist.angular.y = 0;
        hasiru.twist.angular.z = 0;
        hasiru.twist.linear.x = 0;
        hasiru.twist.linear.y = 0;
        hasiru.twist.linear.z = 0;
    }
    std::cout << ror << std::endl;

    twist_pub.publish(hasiru);
    vpDisplay::flush(Idisp);
    shori = true;
    clock_t end = clock();

    //std::cout << "duration = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
}

void Servo_actionserver::kawari_image(sensor_msgs::ImageConstPtr image_msg, study_0 *study_class)
{
    clock_t start = clock();
    shori = false;
    Iin = visp_bridge::toVispImage(*image_msg);
    Idisp.insert(Iin, vpImagePoint(0, 0));
    april_message hantei;
    hantei = study_class->april_detect(image_msg, study_class->Iin, kawari, haji);
    kawari = hantei.cM;
    study_class->cMo_detect = kawari;
    haji = hantei.core;
    Homo cmkd = hantei.cM;
    //study_class->feature_create(hantei.core, study_class->p, hantei.cM);
    study_class->feature_create_kai(4, study_class->p, hantei.cM);
    vpDisplay::display(Idisp);
    PorPd pdf;
    pdf = study_class->pd;
    for (int i = 0; i < pdf.size(); i++) {
        std::stringstream ss;
        vpImagePoint ip;
        ss << i;
        vpMeterPixelConversion::convertPoint(study_class->cam_visp, pdf[i].get_x(), pdf[i].get_y(), ip);
        vpDisplay::displayText(Idisp, ip + vpImagePoint(15, 15), ss.str(), vpColor::green);
        vpDisplay::displayCross(Idisp, ip, 14, vpColor::green, 4);
    }
    pdf = study_class->p;
    for (int i = 0; i < pdf.size(); i++) {
        std::stringstream ss;
        vpImagePoint ip;
        ss << i;
        vpMeterPixelConversion::convertPoint(study_class->cam_visp, pdf[i].get_x(), pdf[i].get_y(), ip);
        vpDisplay::displayText(Idisp, ip + vpImagePoint(15, 15), ss.str(), vpColor::red);
        vpDisplay::displayCross(Idisp, ip, 14, vpColor::red, 4);
    }
    vpColVector vere = study_class->calc_control_law(study_class->task);
    /*for (int i = 0; i < 6; i++) {
        std::cout << study_class->task.computeControlLaw()[i] << " ";
    }
    std::cout << std::endl;*/
    geometry_msgs::TwistStamped hasiru;
    study_class->set_twist(hasiru, vere, study_class->task);


    twist_pub.publish(hasiru);
    vpDisplay::flush(Idisp);
    shori = true;
    clock_t end = clock();

    //std::cout << "duration = " << (double)(end - start) / CLOCKS_PER_SEC << "sec.\n";
}

void Servo_actionserver::parameter_set()
{
    pnh_->getParam("jog_topic_name", jog_topic);
    pnh_->getParam("image_topic_name", image_topic_name);
    pnh_->getParam("camera_info_topic", camera_info_topic);
    pnh_->getParam("mask_image_path", mask_image_file);
    pnh_->getParam("server_name", name_action);
    pnh_->getParam("loop_rate", loop_rate);
    pnh_->getParam("moveit_group", moveit_group);
}

/*
1, Threshould pose vector
2, Current pose vector
*/
bool Servo_actionserver::evaluation_pose_error(pose_message threshold, pose_message current)
{
    if (current.linear_error <= threshold.linear_error && current.anglar_error <= threshold.anglar_error) {
        return true;
    }
    return false;
}
