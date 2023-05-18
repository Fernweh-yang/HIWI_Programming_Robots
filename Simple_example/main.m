% Script for simple kinematic control

clear;
close all;
clc;

include_namespace_dq; 

%% Initialize V-REP interface
vi = DQ_VrepInterface;
vi.disconnect_all(); 
vi.connect('127.0.0.1',19997);
vi.start_simulation();

%% Initialize VREP Robots
fep_vreprobot = FEpVrepRobot('Franka',vi);

%% Load DQ Robotics kinematics
fep  = fep_vreprobot.kinematics();

% maximum joint ranges (deg): (q1..q7)
%       -166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027  -166.0031
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];

%        166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024  166.0031
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

   
goal = [-pi/2.0, 0.004, 0.0, -1.57156, 0.0, 1.57075, 0.0];
xd = fep.fkm(goal);
e = zeros(8,1);
e(1) = 1.0;
disp("Starting control loop:");

while(norm(e)>0.05)
   q = vi.get_joint_positions(fep_vreprobot.joint_names); 
   disp("Current joint positions ");
   q
   x = fep.fkm(q);
   % Task error
   e = vec8(x -xd);
   J = fep.pose_jacobian(q);
   u = -0.1 *J' * e;
   q = q + u;
   disp("------------------");
   fep_vreprobot.send_q_to_vrep(q);
   
end
%% End V-REP
disp("Control finished");
vi.stop_simulation();
vi.disconnect();





