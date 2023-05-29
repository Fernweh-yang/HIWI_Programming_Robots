#include <math.h>
#include <fstream>

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>     // MDH
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>    // DH

#include "franka_ik_He.hpp"
std::vector<std::string> split(std::string s,char ch){
    int start=0;
    int len=0;
    std::vector<std::string> ret;
    for(int i=0;i<s.length();i++){
        if(s[i]==ch){
            ret.push_back(s.substr(start,len));
            start=i+1;
            len=0;
        }
        else{
            len++;
        }
    }
    if(start<s.length())
        ret.push_back(s.substr(start,len));
    return ret;
}

class FEpVrepRobot{
public:

    std::vector<std::string>  joint_names;

    std::string base_frame_name;

    DQ_VrepInterface vrep_interface;

    FEpVrepRobot(std::string,DQ_VrepInterface &);

    void send_q_to_vrep(VectorXd);

    VectorXd get_q_from_vrep();

    DQ_SerialManipulatorMDH kinematics();
};

FEpVrepRobot::FEpVrepRobot(std::string robot_name,DQ_VrepInterface &vi){
    std::string robot_label;
    std::string desired_name="Franka";
    std::string robot_index;
    this->vrep_interface = vi;

    std::vector<std::string> splited_name = split(robot_name,'#');
    robot_label = splited_name[0];
    
    if(robot_label.compare(desired_name)!=0){
        std::cerr << "Franka" << std::endl;
    }

    if(splited_name.size()>1){
        robot_index = splited_name[1];      
    }else{
        robot_index = "";
    }

    int joint_numbers = 8;
    for(int i=1; i<joint_numbers; i++){
        std::string current_joint_name = robot_label + "_joint" + std::to_string(i) + robot_index;
        this->joint_names.push_back(current_joint_name);
    }


    this->base_frame_name = this->joint_names[0];
}

void FEpVrepRobot::send_q_to_vrep(VectorXd q){
    this->vrep_interface.set_joint_positions(this->joint_names,q);
}

VectorXd FEpVrepRobot::get_q_from_vrep(){
    VectorXd q = this->vrep_interface.get_joint_positions(this->joint_names);
    return q;
}

DQ_SerialManipulatorMDH FEpVrepRobot::kinematics(){
    Matrix<double,5,7> FEp_DH_matrix(5,7);
    FEp_DH_matrix <<    0,      0,       0,       0,       0,      0,       0,      //FEp_DH_theta
                    0.333,      0,   0.316,       0,   0.384,      0,   0.107,      //FEp_DH_d
                        0,      0,  0.0825, -0.0825,       0,  0.088,  0.0003,      //FEp_DH_a
                  -M_PI/2, M_PI/2,  M_PI/2, -M_PI/2,  M_PI/2, M_PI/2,       0,      //FEp_DH_alpha
                        0,      0,       0,       0,       0,      0,       0;      //must be 5x7
    DQ_SerialManipulatorMDH kin(FEp_DH_matrix);
    kin.set_reference_frame(this->vrep_interface.get_object_pose(this->base_frame_name));
    kin.set_base_frame(this->vrep_interface.get_object_pose(this->base_frame_name));
    return kin;
}


int main(int argc, char **argv){


    // ********************* start ********************* 
    // ********************* connect to the coppeliasim********************* 
    DQ_VrepInterface vi;
    vi.set_synchronous(true);
    std::string ip = "127.0.0.1";
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    vi.start_simulation();
    // ********************* end ********************* 


    // ********************* start ********************* 
    // ********************* load DQ Robotics kinematics********************* 
    FEpVrepRobot fep_vreprobot("Franka",vi);
    DQ_SerialManipulatorMDH fep = fep_vreprobot.kinematics();
    RowVectorXd q_min(7),q_max(7),goal(7);
    q_min << -2.8973,   -1.7628,   -2.8973,   -3.0718,   -2.8973,   -0.0175,   -2.8973;
    q_max <<  2.8973,    1.7628,    2.8973,   -0.0698,    2.8973,    3.7525,    2.8973;
    goal <<  -M_PI/2.0,   0.004,       0.0,  -1.57156,       0.0,   1.57075,       0.0;
    DQ xd =fep.fkm(goal);
    VectorXd e(8);
    e << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // ********************* end *********************

    // ********************* start ********************* 
    // ********************* Inverse Kinematic********************* 
    /*
    I generate the following matrix by random numbers. But this matrix seems to be invalid, because the result are all nan.
    
    desired_cartesian_pose:
    0.232   0.316   -0.311   -0.32
    -0.434   0.063   0.252   -0.249
    0.22    -0.31    0.452    0.285
    0       0       0        1
    */
    std::array<double, 7> actual_joint_configuration;
    std::array<std::array<double,7>,4> q_all;
    std::array<double, 16> desired_cartesian_pose = {0.232,0.-434,0.22,0,0.316,0.63,-0.31,0,0.-311,0.252,0.452,0,-0.32,-0.249,0.285,1};
    double q7=0;  // the last joint angle
    VectorXd q_current(7);

    q_current = vi.get_joint_positions(fep_vreprobot.joint_names);
    for(int i = 0; i < 7; i++){
        actual_joint_configuration[i] = q_current[i];
        std::cout<< "actual_joint_configuration" << i << ":" << actual_joint_configuration[i] <<std::endl;
    }

    q_all = franka_IK_EE(desired_cartesian_pose,q7,actual_joint_configuration);     // calculate the inverse kinematic

    for(const auto& element: q_all[0]){
        std::cout<< element << " ";
    }
    std::cout<< std::endl;
    for(const auto& element: q_all[1]){
        std::cout << element << " ";
    }
    std::cout<< std::endl;
    for(const auto& element: q_all[2]){
        std::cout << element << " ";
    }
    std::cout<< std::endl;
    for(const auto& element: q_all[3]){
        std::cout << element << " ";
    }
    // ********************* end ********************
    

    std::cout << "Control finished" << std::endl;
    vi.stop_simulation();
    vi.disconnect();
}