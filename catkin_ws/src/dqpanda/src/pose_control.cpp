#include "ros/ros.h"
#include "std_msgs/String.h"

#include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>

#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>

#include <thread>
#include <fstream>

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_control");
    ros::NodeHandle nh;
    
    // connect to the coppeliasim
    DQ_VrepInterface vi;
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int iterations = 1000;


    //------------------- Robot definition--------------------------
    //---------- Franka Emika Panda serial manipulator
    DQ_SerialManipulatorMDH franka = FrankaEmikaPandaRobot::kinematics();

    //Update the base of the robot from CoppeliaSim
    DQ new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(-0.07*k_));
    franka.set_reference_frame(new_base_robot);
    //---------------------------------------------------------------

    std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2",
                                            "Franka_joint3", "Franka_joint4",
                                            "Franka_joint5", "Franka_joint6",
                                            "Franka_joint7"};

    VectorXd vec_torques(7);

    double Kp = 0.01; //4.5
    double Kv = 3*sqrt(Kp);
    VectorXd qd = VectorXd(7);
    qd <<-0.70, -0.10, 1.66, -2.34,0.40, 1.26, 0.070;
    vi.set_object_pose("DesiredFrame", franka.fkm(qd));

    VectorXd g_qd = VectorXd(7);
    Matrix<double, 7,3> list_torques;

    std::ofstream list_torques_ref;
    list_torques_ref.open("list_torques_ref.csv");

    std::ofstream list_torques_read;
    list_torques_read.open("list_torques_read.csv");

    int i=0;
    while(ros::ok() && i<iterations){
        i++;
        VectorXd q = vi.get_joint_positions(jointnames);
        VectorXd qerror = qd-q;
        VectorXd q_dot = vi.get_joint_velocities(jointnames);
        VectorXd qerror_dot = -q_dot;

        vi.set_object_pose("ReferenceFrame", franka.fkm(q));

        vec_torques = Kp*qerror + Kv*qerror_dot ;
        vi.set_joint_torques(jointnames, vec_torques);
        vi.trigger_next_simulation_step();

        VectorXd vec_torques_read = vi.get_joint_torques(jointnames);

        list_torques.col(0) = vec_torques;
        list_torques.col(1) = vec_torques_read;
        list_torques.col(2) = list_torques.col(0)-list_torques.col(1);


        list_torques_ref <<vec_torques(0)<<","<<vec_torques(1)<<","<<vec_torques(2)<<
                           ","<<vec_torques(3)<<","<<vec_torques(4)<<","<<vec_torques(5)<<
                           ","<<vec_torques(6)<<","<<'\n';


        list_torques_read <<vec_torques_read(0)<<","<<vec_torques_read(1)<<","<<vec_torques_read(2)<<
                           ","<<vec_torques_read(3)<<","<<vec_torques_read(4)<<","<<vec_torques_read(5)<<
                           ","<<vec_torques_read(6)<<","<<'\n';


        std::cout<< "torques ref:   torques read:    error:"<<std::endl;
        std::cout<<list_torques<<std::endl;

        std::cout<< "  "<<std::endl;
        std::cout<< "Applying torques..."<<iterations-i<<std::endl;
        std::cout<< "Error..."<<qerror.norm()<<std::endl;
    }
    list_torques_ref.close();
    list_torques_read.close();

    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}