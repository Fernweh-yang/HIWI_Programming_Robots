#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Dense>
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp" // coriolis matrix , gravity contribution
#include "pinocchio/algorithm/crba.hpp" // mass matrix

#include <iostream>
#include <fstream>
// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
    #define PINOCCHIO_MODEL_DIR "../"
#endif

using namespace pinocchio;

int main(int  argc , char **  argv )
{   
    std::ofstream file("dynamic parameters.txt");
    
    const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("panda.urdf") : argv[1];// URDF file path
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename,model);
    std::cout << "model name: " << model.name << std::endl;
    pinocchio::Data data(model);

    const int JOINT_ID = 7;             // The end effector corresponds to the 7th joint
    const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 1., 1.));   //Desired Pose: R + Position
    Eigen::VectorXd q = pinocchio::neutral(model);  // neutral configuration space
    Eigen::VectorXd v(model.nv);        // Velocity vector

    pinocchio::crba(model,data,q);
    std::cout << "Mass Matrix:" << data.M <<std::endl;
    file << "Mass Matrix:" << std::endl;
    file << data.M << std::endl;

    computeCoriolisMatrix(model,data,q,v);
    std::cout << "coriolis matrix" << data.C << std::endl;
    file << "coriolis Matrix:" << std::endl;
    file << data.C << std::endl;

    computeGeneralizedGravity(model,data,q);
    std::cout << "gravity contribution " << data.g << std::endl;
    file << "gravity contribution:" << std::endl;
    file << data.g << std::endl;
   
    file.close();
}