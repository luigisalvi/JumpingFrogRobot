%Main of Frog Kinematics
%Trigger angles as needed. Default values for intial pose are T1=195, T2=165, T3=195.
% 195<=T1<=360
% 165<=T2<=0
% 195<=T3<=360
%% Direct Kinematics validation
%Here different poses are computed to validate the algorithm
clc,clear,close all
plot_trigger = true;
Direct_Kin_frog(plot_trigger,195,165,195)
hold on
Direct_Kin_frog(plot_trigger,200,115,220)
hold on
Direct_Kin_frog(plot_trigger,205,95,235) %*

%% Inverse Kinematics validation

clc,clear,close all
% Disired trajectory generation
trj = 1; %Select 1 for line, 2 for parabola
final_position = [0.4,0.75,0];
path = trajectory_fun(final_position, trj);

%Using inverse kinematics function, all possible combination of angles are
%checked to find the best triplet that match with each desired position.
%The list of desired position are stored in the path vector: for each of
%those points, an inverse kinematics problem is solved and its solutions
%are stored in the joint_positions vector.
for i = 1:length(path)
    plot_trigger=false;
    joint_positions(i,:) = Inverse_Kin_frog(plot_trigger,path(:,i));
end
joint_positions = rad2deg(joint_positions);


figure('Name','Inverse Kinematics path planning')
%Once all triplets of angles needed to follow the trajectory are computed,
%the direct kinematics function for each of these triplets is run, to
%validate the process, verifying that the trajectory is effectively
%followed by thr C.O.M.
for i = 1:length(joint_positions)
    plot_trigger=true;
    Direct_Kin_frog(plot_trigger,joint_positions(i,1),joint_positions(i,2),joint_positions(i,3));
end
hold on
% Trajectory Plot
plot3(path(1,:), path(2,:), path(3,:), '--r', 'LineWidth', 2);
legend off
grid on

%% Theta over Time studies. 
%Remember to run commenting plot function in Direct_Kin_frog.m,
%facilitating the simulation over more parameters. In the trajectory
%function by default trajectories are built on 10 points. To better study
%angles over time is racommended to increase to 100 points.

t = 0:1:99;
plot(t,joint_positions(:,1), t,joint_positions(:,2),t,joint_positions(:,3))
legend('\theta_1','\theta_2','\theta_3')
title('Joint Angles during pre-jumping phase (line trajectory)')

