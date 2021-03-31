% Affiliation: ROAR @ Columbia
% Date:        12/02/2021
 
clear
clc
close all
 
%%
%link Length 
link = [1;1];
% Build your robot 
robot = WindowCleanerArm(link);
t = 3;% time to move
n = 50; %number of segment 
T = linspace(0,t,n); %time array

di = [0;2;0];
df = [2;0;0];

qstr = robot.sltraj(di,df,t,n);
disp(qstr);
% 
% qi = robot.InverseKinematics(di(1),di(2),di(3));
% qf = robot.InverseKinematics(df(1),df(2),df(3));
% qpoly = robot.polytraj(qi,qf,t,n);
% disp(qpoly);
 
% animation
figure
for i = 1:length(T)-1
    robot.setJointAngle(qstr(:,i));
    if i < length(T)
        dt = T(i+1)-T(i);
    end
    robot.animateMotion(dt)
end
% 
% figure
% for i = 1:length(T)-1
%     robot.setJointAngle(qpoly(:,i));
%     if i < length(T)
%         dt = T(i+1)-T(i);
%     end
%     robot.animateMotion(dt)
% end
