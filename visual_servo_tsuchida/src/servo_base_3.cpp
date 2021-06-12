#include <visual_servo_tsuchida/servo_base.h>

/*The file data substitute to std::vector<std::string>*/
std::vector<std::string> study_0::file_input(std::ifstream *in, std::string filename)
{
    in = new std::ifstream(filename);
    std::string line;
    std::getline(*in, line);
    std::vector<std::string> strvec = split(line, ' ');
    return strvec;
}


/*Convert std::vector<std::string> into std::vector<double>*/
std::vector<double> study_0::string_to_double(std::vector<std::string> moji)
{
    std::vector<double> resu;
    for (int i = 0; i < moji.size(); i++) {
        resu.push_back(std::stod(moji[i]));
    }
    return resu;
}

/*The file data substitute to std::vector<double> directly*/
std::vector<double> study_0::input_value(std::ifstream *ifs, std::string name_1)
{
    return string_to_double(file_input(ifs, name_1));
}

/*The std::vector<double> element data export to name path file*/
void study_0::write_value_in_file(std::ofstream *ofs, std::string name, std::vector<double> value)
{
    ofs = new std::ofstream(name);
    for (int i = 0; i < value.size(); i++) {
        *ofs << value[i] << " ";
    }
    *ofs << std::endl;
}

/*The std::vector<double> element data export to name path file in overwrite*/
void study_0::write_value_in_file_overwrite(std::ofstream *ofs, std::string name, std::vector<double> &value)
{
    ofs = new std::ofstream(name, std::ios::in | std::ios::ate);
    //ofs = new std::ofstream(name, std::ios::trunc);
    for (int i = 0; i < value.size(); i++) {
        *ofs << value[i] << " ";
    }
    *ofs << std::endl;
    //value.clear();
}

void study_0::vector_show(std::vector<double> show_vector)
{
    for (int i = 0; i < show_vector.size(); i++) {
        std::cout << show_vector[i] << " ";
    }
    std::cout << std::endl;
}

pose_message study_0::max_Homo_error(Homo cdMoyer, Homo cMoyer)
{
    pose_message error_all;
    pro_show("error calcurate start");
    std::vector<double> cdcd = pose_to_vector(visp_bridge::toGeometryMsgsPose(cdMoyer));
    std::vector<double> cmcm = pose_to_vector(visp_bridge::toGeometryMsgsPose(cMoyer));
    pro_show("Substition is finish");
    if (cdcd.size() != cmcm.size() || cdcd.size() != 6) {
        ROS_ERROR_STREAM("Size is diferrent");
        error_all.anglar_error = 1;
        error_all.linear_error = 1;
        return error_all;
    }
    pro_show("Size is ok");
    double linear_max = std::abs(cdcd[0] - cmcm[0]);
    double tem;
    for (int i = 1; i < 3; i++) {
        tem = std::abs(cdcd[i] - cmcm[i]);
        if (tem > linear_max) {
            linear_max = tem;
        }
    }
    double angular_max = std::abs(cdcd[3] - cmcm[3]);
    for (int i = 4; i < 6; i++) {
        tem = std::abs(cdcd[i] - cmcm[i]);
        if (tem > angular_max) {
            angular_max = tem;
        }
    }
    error_all.linear_error = linear_max;
    error_all.anglar_error = angular_max;
    return error_all;
}