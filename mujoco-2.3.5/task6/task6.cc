#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;

double simulation_endtime = 20;
int step_number;

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// init parameters
VectorXd init_qpos(7);

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


void init_controller(const mjModel* m, mjData* d)
{
  init_qpos << 0.6579, -0.06562,-0.6566,-2.339,-0.0441,2.2911,0.7280;
  for (size_t i=0;i<7;i++)
   {
    d->qpos[i] = init_qpos[i];
  }

}

void init_pose(const mjModel* m, mjData* d){
  init_qpos << 0.6579, -0.06562,-0.6566,-2.339,-0.0441,2.2911,0.7280;
  for (size_t i=0;i<7;i++)
  {
    d->qpos[i] = init_qpos[i];
  }
}

void update_controller(double pos_foot[3], double z_velocity){
  
}

void controller(const mjModel* m, mjData* d)
{
  int body_number = 3;
  double pos_foot[3]={ d->xpos[3*body_number+0], d->xpos[3*body_number+1], d->xpos[3*body_number+2]};
  double z_velocity = d->qvel[1]; //0 is x, 1 is z

  update_controller(pos_foot,z_velocity);

  //all actions
  int actuator_number;
}


int main(int argc, const char** argv)
{
    // ***************** start *****************
    // ***************** load model and data *****************
    mj_activate("mjkey.txt");                 // activate software
    char filepath[] = "../task6/franka_emika_panda//scene.xml";
    char error[1000] = "Could not load binary model";
    
    m = mj_loadXML(filepath, 0, error, 1000); // load and compile model
    if( !m )
        mju_error_s("Load model error: %s", error);

    d = mj_makeData(m);                       // get data of model
    // ***************** end *****************


    // ***************** start *****************
    // ***************** render the window *****************
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
    // create window
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    // initial visual data structure
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                
    mjr_makeContext(m, &con, mjFONTSCALE_150);   
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    // ***************** end *****************


    // ***************** start *****************
    // ***************** define the controller *****************
    // init_pose(m,d);    
    init_qpos << 0.6579,-0.06562,-0.6566,-2.339,-0.0441,2.2911,0.7280;
    for (size_t i=0;i<7;i++){
        d->qpos[i] = init_qpos[i];
    }








    mjcb_control = controller;  // define the control callback;
    // init_controller(m,d);
    // ***************** end *****************


    // ***************** start *****************
    // ***************** simulate *****************
    while( !glfwWindowShouldClose(window))
    {
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

        if (d->time>=simulation_endtime)
        {
           break;
         }

       // view at body
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        int body_number;
        body_number = 1;
        cam.lookat[0] = d->xpos[3*body_number+0];
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    // ***************** end *****************

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();
    return 1;
}

