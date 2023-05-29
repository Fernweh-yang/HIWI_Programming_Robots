#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
    #define PINOCCHIO_MODEL_DIR ""
#endif
int main(int  argc , char **  argv )
{   
    // URDF file path
    const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("panda.urdf") : argv[1];
    pinocchio::Model model;
    // pinocchio::buildModels::manipulator(model);
    pinocchio::urdf::buildModel(urdf_filename,model);
    std::cout << "model name: " << model.name << std::endl;
    pinocchio::Data data(model);

    const int JOINT_ID = 7;             //The end effector corresponds to the 7th joint
    const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1., 1., 1.));   //Desired Pose: R + Position

    Eigen::VectorXd q = pinocchio::neutral(model);  // neutral configuration space
    const double eps  = 1e-4;                       // expected precision
    const int IT_MAX  = 1000;                       // max iterations steps
    const double DT   = 5e-1;                       // convergence rate
    const double damp = 1e-3;                       // damping factor for pseudoinversion

    pinocchio::Data::Matrix6x J(6,model.nv);        // The Jacobian of seven joints
    J.setZero();

    bool success = false;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(model.nv);
    for (int i=0;;i++)
    {
        pinocchio::forwardKinematics(model,data,q);
        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);    // error between the desired pose and the current one R^(4x4)
        err = pinocchio::log6(dMi).toVector();                          // log map: lie group->lie algebra R^6
        if(err.norm() < eps)
        {
        success = true;
        break;
        }
        if (i >= IT_MAX)
        {
        success = false;
        break;
        }
        pinocchio::computeJointJacobian(model,data,q,JOINT_ID,J);   // the jacobian for the end effector
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = - J.transpose() * JJt.ldlt().solve(err);      // 为了避免奇异，所以这里用的伪逆公式，去计算速度。
        q = pinocchio::integrate(model,q,v*DT);                     // 将获得的切向向量加到配置中去，integrate就是简单的加法
        if(!(i%10))
        std::cout << i << ": error = " << err.transpose() << std::endl;
    }

    if(success) 
    {
        std::cout << "Convergence achieved!" << std::endl;
    }
    else 
    {
        std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
    }
        
    std::cout << "\nresult: " << q.transpose() << std::endl;
    std::cout << "\nfinal error: " << err.transpose() << std::endl;
}