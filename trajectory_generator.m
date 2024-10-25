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
    traj_time = arrangeT(path);
    waypoints = path;
    
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
    M = getM(num,7,time_interval);
     
    % TODO:
	% Prepare the selection matrix C.
    Ct = zeros(8 * num, 8 + 2 * (num - 1) + 3 * (num - 1));
    % 为每个航路点设置起始和结束约束
    for i=1:4
        Ct(i,i) = 1;
    end
    for i=1:num-1
        Ct(8*i-3, 2*i+4) = 1;
        Ct(8*i+1, 2*i+4) = 1;
    end
    for i=1:num-1
        for j=2:4
            Ct(8*i-4+j, 8+2*(num-1)+3*(i-1)+j-1) = 1;
        end
    end
    C = Ct';
    
    % TODO: prepare the R matrix of the unconstained Quadric Programming
    % (QP) problem.
    R = C*pinv(M)'*Q*pinv(M)*Ct;
    R_cell = mat2cell(R,[8+2*(num-1), 3*(num-1)],[8+2*(num-1), 3*(num-1)]);
    R_PP = R_cell{2,2};
    R_FP = R_cell{1,2};
    
    % TODO: Solve the unconstrained QP problem.
    % Prepare for d_F
    d_Fx = getdF(path,1);
    d_Fy = getdF(path,2);
    d_Fz = getdF(path,3);
     
    
    % TODO: work out d_P
    d_Px_opt = -pinv(R_PP) * R_FP' * d_Fx;  
    d_Py_opt = -pinv(R_PP) * R_FP' * d_Fy;
    d_Pz_opt = -pinv(R_PP) * R_FP' * d_Fz;
    
    % TODO: stack d_F and d_P into d
    d_x = [d_Fx;d_Px_opt];
    d_y = [d_Fy;d_Py_opt];
    d_z = [d_Fz;d_Pz_opt];
    
    % TODO: mapping back to coefficients of the polynomial (position)
    p_x = pinv(M)*Ct*d_x;
    p_y = pinv(M)*Ct*d_y;
    p_z = pinv(M)*Ct*d_z;
    
    pos_coeffs = [p_x,p_y,p_z];
    
    % TODO: work out the coefficients of the velocity polynomial
    dev_coeffs = repmat((1:7)',num,3);

    vel_coeffs = dev_coeffs .* pos_coeffs(kron(0:num-1,ones(1,7)*8)+kron(ones(1,num),2:8),:);
    
    p_x = reshape(p_x,8,num);p_x = p_x';
    p_y = reshape(p_y,8,num);p_y = p_y';
    p_z = reshape(p_z,8,num);p_z = p_z';
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
%         px = pos_coeffs(:,1:8);
%         py = pos_coeffs(:,9:16);
%         pz = pos_coeffs(:,17:24);
%         
%         vx = vel_coeffs(:,1:8);
%         vy = vel_coeffs(:,9:16);
%         vz = vel_coeffs(:,17:24);
        
        % Calculate the state at the query time instance t
        pos_x = polyval(flip(pos_coeffs((t_index)*8+1:(t_index)*8+8,1)) , t);
        pos_y = polyval(flip(pos_coeffs((t_index)*8+1:(t_index)*8+8,2)) , t);
        pos_z = polyval(flip(pos_coeffs((t_index)*8+1:(t_index)*8+8,3)) , t);
        
        vel_x = polyval(flip(vel_coeffs((t_index)*7+1:(t_index)*7+7,1)) , t);
        vel_y = polyval(flip(vel_coeffs((t_index)*7+1:(t_index)*7+7,2)) , t);
        vel_z = polyval(flip(vel_coeffs((t_index)*7+1:(t_index)*7+7,3)) , t);
        
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


