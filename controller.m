function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

% TODO: declare global containers for restoring RMS errors.
% global rms_xposition;
% ...;
global rms_xposition;
global rms_xvelocity;
global rms_yposition;
global rms_yvelocity;
global rms_zposition;
global rms_zvelocity;


% TODO: read the current states
% s(1:3) current position (i.e., x, y, z)
% s(4:6) current velocity (i.e., dx, dy, dz)
% s(7:10) current attitude quaternion
% s(11:13) current rate of Euler Angles (i.e., dphi, dtheta, dpsi)
% Note that you don't need to know how does quaternion exactly work here.
% -------------------
% To finish this project, you only need to know how to obtain the rotation 
% matrix from a quaternion, and how to recover the Euler angles from a rotation
% matrix. Two useful functions, i.e., quaternion_to_R.m and RotToRPY_ZXY.m have
% been provided in /utils.



% TODO: read the desired states
% s_des(1:3) desire position (i.e., x_des, y_des, z_des)
% s_des(4:6) desire velocity (i.e., dx_des, dy_des, dz_des)
% s_des(7:10) desire attitude quaternion
% s_des(11:13) desire rate of Euler Angles (i.e., dphi, dtheta, dpsi)



% TODO: define and set PD-controller parameters
% kp_xyz = [0,0,0] ;
% kd_xyz = [0,0,0] ;
% An example is as follows.
% kp_angle = [0,0,0] ;
% kd_angle = [0,0,0] ;


% TODO: Position control (calculate u1)



% TODO: Attitude control (calculate u2)
% Rember to deal with angle range or use wrapToPi()

%% 读取当前状态
pos = s(1:3);       % 当前的位置 (x, y, z)
vel = s(4:6);       % 当前的速度 (dx, dy, dz)
quat = s(7:10);     % 当前的姿态四元数
omega = s(11:13);   % 当前的角速度 (dphi, dtheta, dpsi)

[phi,theta,psi] = RotToRPY_ZXY(quaternion_to_R(quat));
angle_cur = [phi;theta;psi];

R_ab = [cos(theta),0,-cos(phi)*sin(theta);0,1,sin(phi);sin(theta),0,cos(phi)*cos(theta)];
W_xyz = R_ab * omega;

% 读取期望状态
pos_des = s_des(1:3);       % 期望的位置 (x_des, y_des, z_des)
vel_des = s_des(4:6);       % 期望的速度 (dx_des, dy_des, dz_des)
quat_des = s_des(7:10);     % 期望的姿态四元数
W_des = s_des(11:13)*0;   % 期望的角速度 (dphi_des, dtheta_des, dpsi_des)
[phiC,thetaC,psiC] = RotToRPY_ZXY(quaternion_to_R(quat_des));

% 位置PD控制器增益
kp_pos = [20; 10; 10];  % 位置比例增益
kd_pos = [20; 30; 20];   % 速度微分增益

% 姿态PD控制器增益
kp_angle = [400; 400; 400];  % 姿态角比例增益
kd_angle = [55; 55; 55];     % 姿态角微分增益

%% 位置控制（计算推力F）
pos_error = pos_des - pos;             % 位置误差
vel_error = vel_des - vel;             % 速度误差
acc_des = kp_pos .* pos_error + kd_pos .* vel_error;   % 期望加速度

% 计算推力 F (假设推力主要作用在 z 轴)
F = m * (g + acc_des(3));              % 只考虑z方向的推力

%% 姿态控制（计算扭矩M）
% 将期望的推力方向转换为期望的姿态
% 期望的推力方向（z 轴向上）
phiC = 1/g*(acc_des(1)*sin(psi)-acc_des(2)*cos(psi));
thetaC = 1/g*(acc_des(1)*cos(psi)+acc_des(2)*sin(psi));
angleC = [phiC;thetaC;psiC];

angle_error = angleC - angle_cur;
angle_error = [angle_error(1);wrapToPi(angle_error(2));wrapToPi(angle_error(3))];

angle_dv2 = kp_angle .* angle_error + kd_angle .* (W_des - omega);

% 计算扭矩 M
M = I * angle_dv2 + cross(W_xyz, I * W_xyz);


%% TODO: Record all RMS of states, including position and velocity, for visualization.
% An example is as follows.
% rms_xposition=[rms_xposition,s_des(1)-s(1)];
rms_xposition=[rms_xposition,s_des(1)-s(1)];
rms_xvelocity=[rms_xvelocity,s_des(4)-s(4)];
rms_yposition=[rms_yposition,s_des(2)-s(2)];
rms_yvelocity=[rms_yvelocity,s_des(5)-s(5)];
rms_zposition=[rms_zposition,s_des(3)-s(3)];
rms_zvelocity=[rms_zvelocity,s_des(6)-s(6)];

end