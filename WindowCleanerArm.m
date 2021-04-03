classdef WindowCleanerArm < handle
    % This class defines a planar manipulator, which has two links. Each
    % link rotates about a revolute joint located at its end.
    % This program solves the kinematics and dynamics of system.
    % Author:      Haohan Zhang
    % Edits: Rand Hidayah and Fitsum E. Petros
    % Affiliation: ROAR @ Columbia


    properties
        Link      % link lengths; a vector.
        COM
        Mass
    end
    properties (Access = private)
        JointAngle    % angles of each joint, a vector
    end

    methods
        % constructor
        function this = WindowCleanerArm(link,com,mass)
            % construct a robot
            if nargin > 0
                this.Link = link;
                this.COM = com;
                this.Mass = mass;
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
            z = L(1)*sin(q(1))*cos(q(2));
            y = L(1)*sin(q(2));
            posA = [x;y;z];
        end
        function posB = calcPosB(this,q)
            % this calculate the position of the far end of the third link

            L = this.Link;
            x = cos(q(1))*(L(1)*cos(q(2)) + L(2)*cos(q(2)+q(3)));
            z = sin(q(1))*(L(1)*cos(q(2)) + L(2)*cos(q(2)+q(3)));
            y = L(1)*sin(q(2)) + L(2)*sin(q(2)+q(3));
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
            q3=atan2(-sqrt(1-D^2),D);
            q2=atan2(y,sqrt(x^2+z^2))-atan2((L2*sin(q3)),(L1+L2*cos(q3)));
            q1 = atan2(z,x);

            %Returns q1 and q2
            q = [q1;q2;q3]; %

            if any(isnan((q)))
                disp("can not reach or is not in the workspace")
                return
            end
        end
        
        function tau = Torque(this,q,dq,ddq)
            % joint variable
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            dq1 = dq(1);
            dq2 = dq(2);
            dq3 = dq(3);
            %robot variable
            a2 = this.Link(1);
            a3 = this.Link(2);
            ac2 = this.Link(1)*this.COM(1);
            ac3 = this.Link(2)*this.COM(2);
            m2 = this.Mass(1);
            m3 = this.Mass(2);
            
            %D matrix
            d11 = (a2^2*m2)/12 + (a2^2*m3)/2 + (a3^2*m3)/12 + (ac2^2*m2)/2 + (ac3^2*m3)/2 + (a2^2*m3*cos(2*q2))/2 + (ac2^2*m2*cos(2*q2))/2 + (ac3^2*m3*cos(2*q2 + 2*q3))/2 + a2*ac3*m3*cos(q3) + a2*ac3*m3*cos(2*q2 + q3);
            d12 = 0;
            d13 = 0;
            d22 = m3*a2^2 + 2*m3*cos(q3)*a2*ac3 + m2*ac2^2 + m3*ac3^2;
            d23 = ac3*m3*(ac3 + a2*cos(q3));
            d33 = ac3^2*m3;
            D = [[d11,d12,d13];[d12,d22,d23];[d13,d23,d33]];
            
            %C matrix
            c11 = -(dq2*(m3*sin(2*q2)*a2^2 + 2*m3*sin(2*q2 + q3)*a2*ac3 + m2*sin(2*q2)*ac2^2 + m3*sin(2*q2 + 2*q3)*ac3^2))/2 - (ac3*dq3*m3*(ac3*sin(2*q2 + 2*q3) + a2*sin(q3) + a2*sin(2*q2 + q3)))/2;
            c12 = (dq1*(m3*sin(2*q2)*a2^2 + 2*m3*sin(2*q2 + q3)*a2*ac3 + m2*sin(2*q2)*ac2^2 + m3*sin(2*q2 + 2*q3)*ac3^2))/2;
            c13 = (ac3*dq1*m3*(ac3*sin(2*q2 + 2*q3) + a2*sin(q3) + a2*sin(2*q2 + q3)))/2;
            c21 = -(dq1*(m3*sin(2*q2)*a2^2 + 2*m3*sin(2*q2 + q3)*a2*ac3 + m2*sin(2*q2)*ac2^2 + m3*sin(2*q2 + 2*q3)*ac3^2))/2;
            c22 = -a2*ac3*dq3*m3*sin(q3);
            c23 = a2*ac3*dq2*m3*sin(q3);
            c31 = -(ac3*dq1*m3*(ac3*sin(2*q2 + 2*q3) + a2*sin(q3) + a2*sin(2*q2 + q3)))/2;
            c32 = -a2*ac3*m3*sin(q3)*(dq2 + dq3);
            c33 = 0;
            C = [[c11,c12,c13];[c21,c22,c23];[c31,c32,c33]];
            
            %Phi vector
            g = 9.8;
            phi1 = g*cos(q1)*(a2*m3*cos(q2) + ac2*m2*cos(q2) + ac3*m3*cos(q2 + q3));
            phi2 = -g*sin(q1)*(a2*m3*sin(q2) + ac2*m2*sin(q2) + ac3*m3*sin(q2 + q3));
            phi3 = -ac3*g*m3*sin(q2 + q3)*sin(q1);
            phi = [phi1;phi2;phi3];
            
            tau = D*ddq+C*dq+phi;
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
            
            r1 = [[1.5;0; 1.25],[0.5;0; 1.25]].';
            r2 = [[0.5;0; 1.25],[0.5;0;-0.25]].';
            r3 = [[0.5;0;-0.25],[1.5;0;-0.25]].';
            r4 = [[1.5;0;-0.25],[1.5;0; 1.25]].';
            
            L1   = [O posA].';
            L2   = [posA posB].';
            L3   = [posB+[0;0;0.25],posB-[0;0;0.25]].';
            hold on
            
            line(r1(:,1),r1(:,2),r1(:,3),'linewidth',2,'color','black');
            line(r2(:,1),r2(:,2),r2(:,3),'linewidth',2,'color','black');
            line(r3(:,1),r3(:,2),r3(:,3),'linewidth',2,'color','black');
            line(r4(:,1),r4(:,2),r4(:,3),'linewidth',2,'color','black');
            
            line(L1(:,1),L1(:,2),L1(:,3),'linewidth',2,'color','b');
            line(L2(:,1),L2(:,2),L2(:,3),'linewidth',2,'color','b');
            line(L3(:,1),L3(:,2),L3(:,3),'linewidth',2,'color','red');
            plot3(O(1),O(2),O(3),'k.','markersize',25);
            plot3(posA(1),posA(2),posA(3),'k.','markersize',15);
            plot3(posB(1),posB(2),posB(3),'k.','markersize',15);
            hold off
        end
        function animateMotion(this,dt)
            % this animates the motion of the robot
            axis([-0.5 2 0 2 -0.5 2]);
            set(gca,'Ydir','reverse')
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
