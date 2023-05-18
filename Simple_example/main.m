% Script for simple kinematic control

clear;
close all;
clc;

%{
   include_namespace_dq: enable the aliases i_, j_, k_, and E_
   
   对偶四元数（Dual Quaternions）：可以理解为元素为四元数的对偶数，同样可以理解为元素为对偶数的四元数。
   对偶数只能表示平移：a+bE, E^2=0
   四元数只能表示3D旋转:a+bi+cj+dk
   对偶四元数结合两者统一的表示了旋转和平移:(a+bi+cj+dk)+E(A+Bi+Cj+Dk)
%}
include_namespace_dq;   


%{
   DQ Robotics和V-REP的交互：具体函数见论文

   DQ_VrepInterface()：DQ和VREP的interface
   connect()：连接到一个V-REP Remote API server
   disconnect_all()：断开所有的Remote API servers连接
   start_simulation()：使用默认异步asynchronous模式和推荐的 5 ms 通信线程周期进行仿真
   
%}
vi = DQ_VrepInterface;
vi.disconnect_all(); 
vi.connect('127.0.0.1',19997);
vi.start_simulation();

%% Initialize VREP Robots
fep_vreprobot = FEpVrepRobot('Franka',vi);   %使用 V-REP 接口的机器人的类必须是 DQ_VrepRobot 的子类，以提供方法 send_q_to_vrep 和 get_q_from_vrep，来发送和接收机器人配置

%% Load DQ Robotics kinematics
fep  = fep_vreprobot.kinematics(); %得到机器人的运动学参数,参考坐标系 和 D-H坐标系

% maximum joint ranges (deg): (q1..q7)
%       -166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027  -166.0031
q_min = [-2.8973   -1.7628   -2.8973   -3.0718   -2.8973   -0.0175  -2.8973];

%        166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024  166.0031
q_max = [ 2.8973    1.7628    2.8973  -0.0698    2.8973    3.7525    2.8973];

   
goal = [-pi/2.0, 0.004, 0.0, -1.57156, 0.0, 1.57075, 0.0];
xd = fep.fkm(goal);  % 计算前向运动学：指根据机器人或刚体的关节参数（例如关节角度）来计算其末端执行器（例如机械臂末端或末端效应器）的位置和姿态的过程。
e = zeros(8,1);
e(1) = 1.0;
disp("Starting control loop:");

% error每个元数的平方和开方后大于0.05，就说明还没到目标位姿
while(norm(e)>0.05)  
   q = vi.get_joint_positions(fep_vreprobot.joint_names); 
   disp("Current joint positions ");
   q
   x = fep.fkm(q);
   % Task error
   e = vec8(x -xd);  % vec4用于四元数的向量表达，vec8用于对偶四元数的表达
   J = fep.pose_jacobian(q);  %计算将配置速度映射到机器人的坐标的位姿的时间导数的雅可比行列式
   u = -0.1 *J' * e; %u:控制输入= gain*雅可比矩阵的复共轭转置*error
   q = q + u;
   disp("------------------");
   fep_vreprobot.send_q_to_vrep(q);
   
end
%% End V-REP
disp("Control finished");
vi.stop_simulation();
vi.disconnect();





