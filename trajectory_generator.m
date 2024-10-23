function s_des = trajectory_generator(t, path, h)
% -------------------------------------------------------------------
% Input:
% t: Query time instance.
% path: A set of waypoints
% h: Handlers for potential visualization.

% Output: 
% s_des: Desired states at query time instance t.

% Function explanation:
% This function contains two parts, implemented respectively
% under "if nargin > 1" and "else".
% The first part implements the trajetory generation given as
% input a set of waypoints.
% The second part returns the desired state at the given query time
% instance.
% -------------------------------------------------------------------

% Some containers for storing data and resulting trajectory coefficients
persistent waypoints traj_time time_interval pos_coeffs vel_coeffs 
s_des = zeros(13,1);
s_des(7:10) = [1;0;0;0];

% Part I
if nargin > 1 % generate a trajectory (given waypoints)
    % TODO: 
    % Assuming the quadrotor flies at unit speed, we calculate
    % each time instance at which the quadrotor passes by each waypoint.
    num = size(path, 1)-1;  % 航路的段数
    time_interval = arrangeT(path);
    
    % TODO: Minimum-snap trajectory 
    % Prepare the Q matrix for a 7th-order polynomial
    Q = zeros(8 * num, 8 * num);
    for i = 1:num
        T = time_interval(i);  % 获取该段航路的时间间隔
        for r = 4:7  % snap 是 7阶多项式的四阶导数
            for c = 4:7
                Q((i-1)*8+r+1, (i-1)*8+c+1) = (factorial(r)/factorial(r-4)) * (factorial(c)/factorial(c-4)) * T^(r+c-7)/(r+c-7);
            end
        end
    end
    
    % TODO: 
    % Prepare the mapping matrix (polynomial coefficients --> derivatives of states)
    M = zeros(8 * num, 8 * num );
    for i = 1:num
        T =time_interval(i);
        M((i-1)*8+1:(i-1)*8+4, (i-1)*8+1:(i-1)*8+8) = ...
            [1, T, T^2, T^3, T^4, T^5, T^6, T^7;   % 位置
             0, 1, 2*T, 3*T^2, 4*T^3, 5*T^4, 6*T^5, 7*T^6;  % 速度
             0, 0, 2, 6*T, 12*T^2, 20*T^3, 30*T^4, 42*T^5;  % 加速度
             0, 0, 0, 6, 24*T, 60*T^2, 120*T^3, 210*T^4];   % jerk
    end
     
    % TODO:
	% Prepare the selection matrix C.
    C = zeros(8 * num, 8 + 2 * (num - 1) + 3 * (num - 1));
    % 为每个航路点设置起始和结束约束
    for i = 1:(n-1)
        C((i-1)*8+1, i) = 1;  % 起点约束
        C(i*8, i+1) = 1;      % 终点约束
    end
    
    % TODO: prepare the R matrix of the unconstained Quadric Programming
    % (QP) problem.
    R = C' * Q * C;  % 构建优化问题的代价函数
    R_PP = R(1:n, 1:n);  % 位置部分
    R_FP = R(n+1:end, n+1:end);  % 速度、加速度部分
    
    % TODO: Solve the unconstrained QP problem.
    % Prepare for d_F
    d_Fx = zeros(2 * num + 6,1);
    d_Fy = zeros(2 * num + 6,1);
    d_Fz = zeros(2 * num + 6,1);
    d_Fx = [path(:,1); 0; 0];  % 假设起点和终点的速度、加速度为0
    d_Fy = [path(:,2); 0; 0];
    d_Fz = [path(:,3); 0; 0];
    
    
    % TODO: work out d_P
    d_Px_opt = -inv(R_PP) * R_FP' * d_Fx;  % 位置
    d_Py_opt = -inv(R_PP) * R_FP' * d_Fy;
    d_Pz_opt = -inv(R_PP) * R_FP' * d_Fz;
    
    % TODO: stack d_F and d_P into d
%     d_x = 
%     d_y = 
%     d_z = 
    
    % TODO: mapping back to coefficients of the polynomial (position)
%     p_x = 
%     p_y = 
%     p_z = 
    
%     pos_coeffs = 
    
    % TODO: work out the coefficients of the velocity polynomial
    v_x = zeros(size(p_x));
    v_y = zeros(size(p_y));
    v_z = zeros(size(p_z));

    % ...
    
    vel_coeffs = [v_x v_y v_z];
    
    % Visualization: plot the trajectory
    subplot(h);
    pos_x = [];
    pos_y = [];
    pos_z = [];
    for i = 1 : num
        T = time_interval(i);
        t = 0:0.01:T;
        pos_xi = polyval(flip(p_x(i,:)),t);
        pos_yi = polyval(flip(p_y(i,:)),t);
        pos_zi = polyval(flip(p_z(i,:)),t); 
        pos_x = [pos_x pos_xi];
        pos_y = [pos_y pos_yi];
        pos_z = [pos_z pos_zi];
    end 
    plot3(pos_x, pos_y, pos_z, 'r-','LineWidth',2);
    
% Part II: Output desired states at the query time instance
else 
% First, check whether out of range? If so, adjust to the final time.
    if t > traj_time(end)
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1) - 1;  
    
% Second, deal with the sitation when t == 0
    if t == 0
        s_des(1:3) = path(1,:);
        s_des(7:10) = [1;0;0;0];
% Third, deal with the normal case
    else
        if t_index > 1
            t  = t - traj_time(t_index);
        end
       
        % load coefficients from the resulting trajectory
        px = pos_coeffs(:,1:8);
        py = pos_coeffs(:,9:16);
        pz = pos_coeffs(:,17:24);
        
        vx = vel_coeffs(:,1:8);
        vy = vel_coeffs(:,9:16);
        vz = vel_coeffs(:,17:24);
        
        % Calculate the state at the query time instance t
        pos_x = polyval(flip(px(t_index,:)) , t);
        pos_y = polyval(flip(py(t_index,:)) , t);
        pos_z = polyval(flip(pz(t_index,:)) , t);
        
        vel_x = polyval(flip(vx(t_index,:)) , t);
        vel_y = polyval(flip(vy(t_index,:)) , t);
        vel_z = polyval(flip(vz(t_index,:)) , t);
        
        % Output
        s_des(7:10) = [1;0;0;0];
        s_des(1) = pos_x;
        s_des(2) = pos_y;
        s_des(3) = pos_z;
        
        s_des(4) = vel_x;
        s_des(5) = vel_y;
        s_des(6) = vel_z; 
    end     
end

end


