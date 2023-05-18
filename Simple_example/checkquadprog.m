% Script for simple quadprog

clear;
close all;
clc;
H = [1 -1; -1 2]; 
f = [-2; -6];
A = [1 1; -1 2; 2 1];
b = [2; 2; 3];
i = 0;
Aeq = [];
beq = [];
lb = [];
ub = [];
x0 = [];
options = optimset('Display', 'off');
while(i <= 10)
   [x,fval,exitflag,output,lambda] = ...
   quadprog(H,f,A,b,Aeq,beq,lb,ub,x0,options);
   i = i +1;
   disp(i)
end
