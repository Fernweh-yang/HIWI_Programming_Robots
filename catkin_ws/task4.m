clear; 
close all; 
clc;

data = readmatrix('joint_pose_log.csv');  

steps         = data(:, 1);
Franka_joint1 = data(:, 2);
Franka_joint2 = data(:, 3);  
Franka_joint3 = data(:, 4);  
Franka_joint4 = data(:, 5);  
Franka_joint5 = data(:, 6);  
Franka_joint6 = data(:, 7);  
Franka_joint7 = data(:, 8);    
error         = data(:, 9);

figure;
plot(steps, Franka_joint1, 'r-', 'LineWidth', 2);  
hold on;  
plot(steps, Franka_joint2, 'g--', 'LineWidth', 2);  
plot(steps, Franka_joint3, 'b-.', 'LineWidth', 2);  
plot(steps, Franka_joint4, 'm-', 'LineWidth', 2);  
plot(steps, Franka_joint5, 'c--', 'LineWidth', 2);  
plot(steps, Franka_joint6, 'k-.', 'LineWidth', 2);  
plot(steps, Franka_joint7, 'y-', 'LineWidth', 2);  
plot(steps, error, 'ro--', 'LineWidth', 2);  


legend('Franka_joint1', 'Franka_joint2', 'Franka_joint3', 'Franka_joint4', 'Franka_joint5', 'Franka_joint6', 'Franka_joint7','error');  
xlabel('steps');  
title('Joint position and error');  