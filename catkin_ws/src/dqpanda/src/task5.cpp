#include <ros/ros.h>
#include <std_msgs/String.h>
#include <math.h>

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>     // MDH
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>    // DH

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

// get the template method from https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{

	// Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV); // For a square matrix
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);    // For a non-square matrix
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "task_3_4");
    ros::NodeHandle nh;

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
    std::cout << "Debug succeed:" <<std::endl;
    RowVectorXd q_min(7),q_max(7),goal(7);
    q_min << -2.8973,   -1.7628,   -2.8973,   -3.0718,   -2.8973,   -0.0175,   -2.8973;
    q_max <<  2.8973,    1.7628,    2.8973,   -0.0698,    2.8973,    3.7525,    2.8973;
    goal <<  -M_PI/2.0,   0.004,       0.0,  -1.57156,       0.0,   1.57075,       0.0;
    DQ xd =fep.fkm(goal);
    VectorXd e(8);
    e << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // ********************* end *********************


    // ********************* start ********************* 
    // ********************* Control Loop********************* 
    DQ x;
    MatrixXd J,J_pinv,JJ;
    VectorXd q,u;
    double gain = -1.5;
    std::cout << "Starting control loop:" <<std::endl;
    // >>>>>>>>>> task 5 begin >>>>>>>>>>
    double d = 0.1;
    double k = 0.1;
    RowVectorXd q_c;
    q_c = 0.5*(q_min+q_max);
    // <<<<<<<<<< task 5 end <<<<<<<<<<

    
    while(e.norm()>0.05){
        q = vi.get_joint_positions(fep_vreprobot.joint_names);
        std::cout << "current joint positions: " << std::endl;
        std::cout << q <<std::endl;
        x = fep.fkm(q);
        e = vec8(x-xd);
        J = fep.pose_jacobian(q);

        // >>>>>>>>>> task 5 begin >>>>>>>>>>
        J_pinv = pseudoInverse(J);
        JJ = J_pinv*J;
        MatrixXd I =MatrixXd::Identity(JJ.rows(),JJ.cols());
        MatrixXd N = I - JJ;
        u = -J_pinv*k*e+N*d*(q_c.transpose()-q);
        // <<<<<<<<<< task 5 end <<<<<<<<<<
        q = q + u;
        std::cout << "---------------------- " << std::endl;
        fep_vreprobot.send_q_to_vrep(q);
    }
    // ********************* end *********************

    std::cout << "Control finished" << std::endl;
    vi.stop_simulation();
    vi.disconnect();
}