% Affiliation: ROAR @ Columbia
% Date:        12/02/2021
 
clear
clc
close all
 
%%
%link Length, since first joint and second joint overlap, mass and link
%size are omited
link = [1;1];
com = [0.5;0.5];
mass = [0.5;0.5];
% Build your robot 
robot = WindowCleanerArm(link,com,mass);
t = 3;% time to move
n = 350; %number of segment 
T = linspace(0,t,n); %time array

% Define the working area, right now it is just a rectangle 
% (4 variable representing the 4 boundary of rectangle).
% The furthest point in the rectangle to the origin should not be more than 2
% Avoid x=y=0 which is singularity
top = 1.5;
bot = 0.5;
left = 0;
right = 1;

% predetermined washer path
guide = [[left;0;top],[right;0;top],[left;0;top],[left;0;(top+bot)/2],[right;0;(top+bot)/2],[left;0;(top+bot)/2],[left;0;bot],[right;0;bot],[left;0;bot]];
time = [50,50,25,50,50,25,50,50];
ptime = [0,50,100,125,175,225,250,300];
q = zeros(3,n);
for i = 1:length(guide)-1
    coefficient = linspace(0,1,time(i));
    set = guide(:,i) + coefficient.*(guide(:,i+1) - guide(:,i));
    for j = 1:time(i)
        q(:,ptime(i)+j) = robot.InverseKinematics(set(1,j), set(2,j), set(3,j));
    end
end

% getting "fake" velocity and acceleration
dq = zeros(3,n);
% dq(:,1) = (q(:,2)-q(:,1))/(T(2)-T(1));
% dq(:,n) = (q(:,n)-q(:,n-1))/(T(n)-T(n-1));
for i = 2:n-1
    dq(:,i) = (q(:,i+1)-q(:,i-1))/(T(i+1)-T(i-1));
end

ddq = zeros(3,n);
% dq(:,1) = (q(:,2)-q(:,1))/(T(2)-T(1));
% dq(:,n) = (q(:,n)-q(:,n-1))/(T(n)-T(n-1));
for i = 2:n-1
    ddq(:,i) = (dq(:,i+1)-dq(:,i-1))/(T(i+1)-T(i-1));
end

% Torque time series
tau = zeros(3,n);
for i = 1:n
    tau(:,i) = robot.Torque(q(:,i),dq(:,i),ddq(:,i));
end

% Torque plot
figure
plot(T,transpose(tau));
legend('q1','q2','q3')
 
% animation
figure
for i = 1:length(T)-1
    robot.setJointAngle(q(:,i));
    if i < length(T)
        dt = T(i+1)-T(i);
    end
    robot.animateMotion(dt,top,bot,left,right)
end
