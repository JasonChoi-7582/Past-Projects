%% 2-link dynamics w/ cart
close all
clc

global mcart mlink1 mlink2 l1 l2 lcenter1 lcenter2 g I1 I2 
global pos_ref dpos_ref theta1_ref dtheta1_ref theta2_ref dtheta2_ref

mcart = 15; mlink1 = 12.5; mlink2 = 19.4; l1 = 0.75; %length of link 1
l2 = 0.9; %length of link 2
lcenter1 = 0.5; %COM for link 1
lcenter2 = 0.2; %COM for link 2
g = 9.81; %gravity
I1 = 8.12; %Inertia for Link 1
I2 = 7.25; %Intertia for Link 2
t_sim = 0: 0.01: 10; %time for simulation
n = length(t_sim);

%starting position for the cart w/ link
pos1 = 0;
theta1 = -90*pi/180;
theta2 = -90*pi/180; 
x_initial = [pos1; 0 ; theta1; 0; theta2; 0]; %initial positing/angle matrix

pos_ref = 0.0; %initial reference position
dpos_ref = 0.0; %initial reference velocity
theta1_ref = 0.0*pi/180; %initial reference angle for link 1
dtheta1_ref = 0.0*pi/180; %initial reference angular velocity for link 1
theta2_ref = 0.0*pi/180; %initial reference angle for link 2 
dtheta2_ref = 0.0*pi/180; %initial reference angular velocity for link 2

%pos/theta1/theta2 reference
pos_ref_vec = zeros(length(t_sim),1);
theta1_ref_vec = zeros(length(t_sim),1);
theta2_ref_vec = zeros(length(t_sim),1);

%pos/theta1/theta2 reference for t=2
pos_ref_vec(201:400) = 3;
theta1_ref_vec(201:400) = -120*pi/180;
theta2_ref_vec(201:400) = -130*pi/180;

%pos/theta1/theta2 reference for t=4
pos_ref_vec(401:600) = 5;
theta1_ref_vec(401:600) = -90*pi/180;
theta2_ref_vec(401:600) = -90*pi/180;

%pos/theta1/theta2 reference for t=6
pos_ref_vec(601:800) = 2;
theta1_ref_vec(601:800) = 50*pi/180;
theta2_ref_vec(601:800) = 55*pi/180;

%pos/theta1/theta2 reference for t=8
pos_ref_vec(801:end) = 4;
theta1_ref_vec(801:end) = -120*pi/180;
theta2_ref_vec(801:end) = -35*pi/180;

%solve ODE
[t,x] = ode45('forward_dynamics', t_sim, x_initial);

%Plot graph of simulation v. Reference
SIM_posx = x(:,1); %plot for position of cart
SIM_theta1 = x(:,3); %plot for position of link 1
SIM_theta2 = x(:,5); %plot for position of link 2
plot(t,x(:,1), 'b',t,x(:,3), 'g', t,x(:,5), 'r'); hold on; %plot graph time v rad
plot(t, pos_ref_vec, 'k', t, theta1_ref_vec, 'k', t, theta2_ref_vec, 'k');
xlabel('time(s)'), ylabel('position/rad')
legend('cart','\theta_{1}','\theta_{2}')
title('Reference position/angle v. Simulation Results')
grid on

% Reference length and axes
link1_x = SIM_posx-l1*cos(SIM_theta1);
link1_y = -l1*sin(SIM_theta1);
link2_x = SIM_posx-l1*cos(SIM_theta1) + l2*cos(SIM_theta2);
link2_y = -l1*sin(SIM_theta1) - l2*sin(SIM_theta2);
Cart_Center = SIM_posx;


%% Animation
fig1=figure();
CART_Bottom = plot([Cart_Center(1)-2, Cart_Center(1)+2], [-1,-1], 'LineWidth', 3); hold on;
CART_Top = plot([Cart_Center(1)-2, Cart_Center(1)+2], [1,1], 'LineWidth', 3); hold on;
CART_Left = plot([Cart_Center(1)-2, Cart_Center(1)-2], [-1,1], 'LineWidth',3); hold on;
CART_Right = plot([Cart_Center(1)+2, Cart_Center(1)+2], [-1,1], 'LineWidth',3); hold on;
LINK1 = plot([Cart_Center(1),link1_x(1)],[0,link1_y(1)],'LineWidth',3); hold on;
LINK2 = plot([link1_x(1),link2_x(1)],[link1_y(1),link2_y(1)],'LineWidth',3); hold on;
JOINT = plot(link1_x(1),link1_y(1),'o','MarkerSize',10); hold on;
EE = plot(link2_x(1), link2_y(1), 'x', 'MarkerSize', 10); hold on;
JOINT_CART = plot(Cart_Center(1),0,'o','MarkerSize',10); hold on;
BasePoint = plot([-5,10], [0,0], 'Linewidth', 5); hold on;
ax = [-3 7.5 -3 3]; % Walking
grid on
axis('equal',ax)
 
for cnt = 1:2:n
     figure(fig1)
     clf(fig1);
     CART_Bottom = plot([Cart_Center(cnt)-2, Cart_Center(cnt)+2], [-1,-1], 'k', 'LineWidth', 3); hold on;
     CART_Top = plot([Cart_Center(cnt)-2, Cart_Center(cnt)+2], [1,1], 'k', 'LineWidth', 3); hold on;
     CART_Left = plot([Cart_Center(cnt)-2, Cart_Center(cnt)-2], [-1,1], 'k', 'LineWidth',3); hold on;
     CART_Right = plot([Cart_Center(cnt)+2, Cart_Center(cnt)+2], [-1,1], 'k', 'LineWidth',3); hold on;
     LINK1 = plot([Cart_Center(cnt),link1_x(cnt)],[0,link1_y(cnt)], 'b', 'LineWidth',3); hold on;
     LINK2 = plot([link1_x(cnt),link2_x(cnt)],[link1_y(cnt),link2_y(cnt)], 'c', 'LineWidth',3); hold on;
     JOINT = plot(link1_x(cnt),link1_y(cnt),'o','MarkerSize',10); hold on;
     JOINT_CART = plot(Cart_Center(cnt),0,'o','MarkerSize',10); hold on;
     EE = plot(link2_x(cnt), link2_y(cnt), 'x', 'MarkerSize', 10); hold on;
     BasePoint = plot([-5,10], [0,0], 'g', 'Linewidth', 5); hold on;
     title('2 Link Dynamics with Cart');
     ax = [-3 7.5 -3 3]; % Walking
     axis('equal',ax)
     grid on
     drawnow;
end