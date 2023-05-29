#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Dense>
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/dynamics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp" // coriolis matrix , gravity contribution
#include "pinocchio/algorithm/crba.hpp" // mass matrix

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
    #define PINOCCHIO_MODEL_DIR "../"
#endif

using namespace pinocchio;

int main(int  argc , char **  argv )
{   
    const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("panda.urdf") : argv[1];// URDF file path
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename,model);
    std::cout << "model name: " << model.name << std::endl;
    pinocchio::Data data(model);

    const int JOINT_ID = 7;             // The end effector corresponds to the 7th joint
    const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 1., 1.));   //Desired Pose: R + Position
    Eigen::VectorXd q = pinocchio::neutral(model);  // neutral configuration space
    Eigen::VectorXd q_desired,error,q_desired_tt,error_t;
    Eigen::VectorXd v(model.nv);        // Velocity vector
    Eigen::VectorXd tau(model.nq);      // force

    error = q_desired-q;
    double kd = 0.1;
    double kp = 0.1;
    double dt = 0.1;
    while(error.norm()>0.05){
        forwardKinematics(model,data,q);            // update the q
        pinocchio::crba(model,data,q);              // mass matrix
        computeCoriolisMatrix(model,data,q,v);      // coriolis matrix
        computeGeneralizedGravity(model,data,q);    // gravity contribution
        q_desired_tt = q_desired/(dt*dt);
        error_t = error/dt;
        tau = data.M*(q_desired_tt+kd*error_t+kp*error) + data.C + data.g; //control law
        forwardDynamics(model,data,q,v,tau);        // update force tau
    }
}