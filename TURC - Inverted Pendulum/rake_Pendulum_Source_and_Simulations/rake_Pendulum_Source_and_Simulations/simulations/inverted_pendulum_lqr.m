%Original code from
% http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace
%
%Modified by: Nathanael Rake
clear;
clc;

%M = 1.789;    %Mass of cart
M = 20.00;
m = 1.36;    %Mass of pendulum
b = 0.9;    %Damping coefficient
I = 0.1963;  %Pendulum moment of inertia
g = 9.8;    %gravity
l = 0.301;    %distance to pendulum center of mass

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
 
%LQR Control
R = 1;      %Effort
q1 = 1000000;     %Position
q2 = 0;     %Velocity
q3 = 500000;     %Angle
q4 = 0;     %Angular Velocity
Q = [
    q1, 0, 0, 0;
    0, q2, 0, 0;
    0, 0, q3, 0;
    0, 0, 0, q4;
    ];

K = lqr(A,B,Q,R)'

%Closed Loop System
Acl = A-B*K';
Bcl = B;
Ccl = C;
Dcl = D;

sys_cl = ss(Acl,Bcl,Ccl,Dcl);

%Reference Scaling
Cpos = [1, 0, 0, 0];
Dpos = 0;

Nvec = [A, B; Cpos, Dpos]^-1 * [zeros(length(A),1);1];
Nx = Nvec(1:length(A));
Nu = Nvec(length(A)+1);
rScale = Nu+K'*Nx

%Reference Signal
t = 0:0.01:10;
r = 0.5*ones(size(t));

%System Response
[y,t,x]=lsim(sys_cl,r,t); %wit x0 = 0
x0 = [0.1,0.1,0.1,0.1];
%[y,t,x]=lsim(sys_cl,r,t,x0);

f = x*K;

%Plot
% figure(1)
% subplot(2,2,1)
% plot(t,y(:,1))
% title('Position')
% 
% subplot(2,2,2)
% plot(t,y(:,2))
% title('Angle')
% 
% 
% subplot(2,2,3)
% plot(t,f)
% title('Force')

