%Original code from
% http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace
%
%Modified by: Nathanael Rake

M = 1.0;    %Mass of cart
m = 0.5;    %Mass of pendulum
b = 0.0;    %Damping coefficient
I = 0.006;  %Pendulum moment of inertia
g = 9.8;    %gravity
l = 0.5;    %distance to pendulum center of mass

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

%State Space Representation
A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];
 
 openLoopSS = ss(A,B,C,D);

%Open loop system poles
%The poles of the system determine its response
openLoopPoles = eig(A);

%Open loop response to initial conditions
ic = [0, 0.25, 0, 1];
tf = 2;
[yOL,tOL,xOL]=initial(openLoopSS,ic,tf);

figure(1);
    subplot(2,1,1);
        plot(tOL,yOL(:,1));
        title('Cart Position vs. Time');
        xlabel('Time (s)');
        ylabel('Position (m)')
    subplot(2,1,2);
        plot(tOL,yOL(:,2));
        title('Pendulum Angle vs. Time');
        xlabel('Time (s)');
        ylabel('Angle (rad)');

%Desired poles (i.e., desired system response)
desiredPoles = [-5,-6,-7,-8];

%Gains required to achieve this response
K = place(A,B,desiredPoles)';

%New state space representation
Anew = A-B*K';
Bnew = [0, 0, 0, 0]';
Cnew = C;
Dnew = D;

closedLoopSS = ss(Anew,Bnew,Cnew,Dnew);

%Closed loop response to initial conditions
[yCL,tCL,xCL]=initial(closedLoopSS,ic,tf);

figure(2);
    subplot(2,1,1);
        plot(tCL,yCL(:,1));
        title('Cart Position vs. Time');
        xlabel('Time (s)');
        ylabel('Position (m)')
    subplot(2,1,2);
        plot(tCL,yCL(:,2));
        title('Pendulum Angle vs. Time');
        xlabel('Time (s)');
        ylabel('Angle (rad)');