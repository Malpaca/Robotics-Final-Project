classdef WindowCleanerArm < handle
    % This class defines a planar manipulator, which has two links. Each
    % link rotates about a revolute joint located at its end.
    % This program solves the kinematics and dynamics of system.
    % Author:      Haohan Zhang
    % Edits: Rand Hidayah and Fitsum E. Petros
    % Affiliation: ROAR @ Columbia


    properties
        Link      % link lengths; a vector.
    end
    properties (Access = private)
        JointAngle    % angles of each joint, a vector
    end

    methods
        % constructor
        function this = WindowCleanerArm(link)
            % construct a robot
            if nargin > 0
                this.Link = link;
            end
        end

        % setters
        function setJointAngle(this,value)
            % this assigns the joint angles
            this.JointAngle = value;
        end




        % getters
        function value = getJointAngle(this)
            % this returns the joint angles
            value = this.JointAngle;
        end

        % helpers
        function posA = calcPosA(this,q)
            % this calculate the postion of the tip of the second link

            L = this.Link;
            x = L(1)*cos(q(1))*cos(q(2));
            y = L(1)*sin(q(1))*cos(q(2));
            z = L(1)*sin(q(2));
            posA = [x;y;z];
        end
        function posB = calcPosB(this,q)
            % this calculate the position of the far end of the third link

            L = this.Link;
            x = cos(q(1))*(L(1)*cos(q(2)) + L(2)*cos(q(2)+q(3)));
            y = sin(q(1))*(L(1)*cos(q(2)) + L(2)*cos(q(2)+q(3)));
            z = L(1)*sin(q(2)) + L(2)*sin(q(2)+q(3));
            posB = [x;y;z];
        end

        function q = InverseKinematics(this,x,y,z)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % FILL THIS OUT: get expressions for q1 and q2 given x and y
            % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            L = this.Link;
            L1 = L(1);
            L2 = L(2);
            D = (x^2+y^2+z^2-L1^2-L2^2)/(2*L1*L2);
            q3=atan2(sqrt(1-D^2),D);
            q2=atan2(z,sqrt(x^2+y^2))-atan2((L2*sin(q3)),(L1+L2*cos(q3)));
            q1 = atan2(y,x);

            %Returns q1 and q2
            q = [q1;q2;q3]; %

            if any(isnan((q)))
                disp("can not reach or is not in the workspace")
                return
            end
        end

        function q = polytraj(this,qi, qf,t,n)
            % computes third degree poly trajectory
            q1 = [qi(1) 0 qf(1) 0];
            q2 = [qi(2) 0 qf(2) 0];
            q3 = [qi(3) 0 qf(3) 0];
            T = linspace(0,t,n);
            M = [1 0 0 0; 0 1 0 0; 1 t t^2 t^3; 0 1 2*t 3*t^2];
            coffq1 = M\q1';
            coffq2 = M\q2';
            coffq3 = M\q3';
            q1t = coffq1(1) +coffq1(2).*T+ coffq1(3).*T.^2 + coffq1(4).*T.^3;
            q2t = coffq2(1) +coffq2(2).*T+ coffq2(3).*T.^2 + coffq2(4).*T.^3;
            q3t = coffq3(1) +coffq3(2).*T+ coffq3(3).*T.^2 + coffq3(4).*T.^3;
            q = [q1t;q2t;q3t];
        end
        function q = sltraj(this,di,df,t,n)
            % computes straightline trajectory
            coefficient = linspace(0,1,n);
            set = di + coefficient.*(df-di);
            q = zeros(3,n);
            for i = 1:n
                q(:,i) = this.InverseKinematics(set(1,i), set(2,i), set(3,i));
            end
        end
        function plotRobot(this)
            % this plots the geometry of robot
            O    = [0;0;0];
            q    = this.getJointAngle;
            posA = this.calcPosA(q);
            posB = this.calcPosB(q);
%             disp("q:");
%             disp(q);
%             disp("joint 2");
%             disp(posA);
%             disp("joint 3");
%             disp(posB);
            L1   = [O posA].';
            L2   = [posA posB].';
            hold on
            line(L1(:,1),L1(:,2),L1(:,3),'linewidth',2,'color','b');
            line(L2(:,1),L2(:,2),L2(:,3),'linewidth',2,'color','b');
            plot3(O(1),O(2),O(3),'k.','markersize',25);
            plot3(posA(1),posA(2),posA(3),'k.','markersize',25);
            plot3(posB(1),posB(2),posB(3),'k.','markersize',25);
            hold off
        end
        function animateMotion(this,dt)
            % this animates the motion of the robot
            axis([-2 2 -2 2 -2 2]);
            view(3);
            grid on;
            xlabel('x');
            ylabel('y');
            zlabel('z');
            this.plotRobot;
            drawnow
            pause(dt) % pause with a 'correct' timing
            clf
        end
    end

end