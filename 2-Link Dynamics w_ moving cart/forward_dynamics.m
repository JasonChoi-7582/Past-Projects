%% Forward and Inverse Dynamics for 2link w/ cart
function x_dot = forward_dynamics(t, X)

global mcart mlink1 mlink2 l1 g I1 I2 lcenter1 lcenter2
global pos_ref dpos_ref theta1_ref dtheta1_ref theta2_ref dtheta2_ref

posx = X(1);
dposx = X(2);
theta1 = X(3);
dtheta1 = X(4);
theta2 = X(5);
dtheta2 = X(6);

%Relative position versus time
if abs(t-2.0) < 0.1
    pos_ref = 3;
    theta1_ref = -120*pi/180;
    theta2_ref = -130*pi/180;
elseif abs(t-4.0) < 0.1
    pos_ref = 5;
    theta1_ref = -90*pi/180;
    theta2_ref = -90*pi/180;
elseif abs(t-6.0) < 0.1
    pos_ref = 2;
    theta1_ref = 50*pi/180;
    theta2_ref = 55*pi/180;
elseif abs(t-8.0) <0.1 
    pos_ref = 4;
    theta1_ref = -120*pi/180;
    theta2_ref = -35*pi/180;
end

%Inverse Dynamics using PD control
Kp1 = 450.0; Kd1 = 250.0;
Kp2 = 1000.0; Kd2 = 250.0;
Kp3 = 1000.0; Kd3 = 250.0;
 
force1 = Kp1*(pos_ref - posx) + Kd1*(dpos_ref - dposx);
torque2 = Kp3*(theta2_ref-theta2)+Kd3*(dtheta2_ref-dtheta2)+mlink2*lcenter2*g*sin(theta2);
torque1 = torque2 + Kp2*(theta1_ref-theta1)+Kd2*(dtheta1_ref-dtheta1)+(mlink1*lcenter1 + mlink2*l1)*g*sin(theta1);

%Forward Dynamics using Lagrangian Mechanics
M = [mcart+mlink1+mlink2 (mlink1*lcenter1+mlink2*l1)*cos(theta1) mlink2*lcenter2*cos(theta2); (mlink1*lcenter1+mlink2*l1)*cos(theta1) mlink1*lcenter1^2+mlink2*l1^2+I1 mlink2*l1*lcenter2*cos(theta1-theta2); mlink2*lcenter2*cos(theta2) mlink2*l1*lcenter2*cos(theta1-theta2) mlink2*lcenter2^2*I2]; %Euler
C = [0 -(mlink1*lcenter1+mlink2*l1)*sin(theta1)*dtheta1 -mlink2*lcenter2*sin(theta2)*dtheta2; 0 0 mlink2*l1*lcenter2*sin(theta1-theta2)*dtheta2; 0 -mlink2*l1*lcenter2*sin(theta1-theta2)*dtheta1 0]; %Corolius force
G = [0; -(mlink1*lcenter1 + mlink2*l1)*g*sin(theta1); -mlink2*lcenter2*g*sin(theta2)]; %Gravity force
S=[1 0 0; 0 1 -1; 0 0 1];
K=M\(C+G+S*[force1;torque1;torque2]); %Lagrange equation

%final result
ddposx=K(1,1);
ddtheta1=K(2,1);
ddtheta2=K(3,1);
x_dot = [dposx; ddposx; dtheta1; ddtheta1; dtheta2; ddtheta2];
end
