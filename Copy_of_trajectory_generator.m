function s_des = trajectory_generator(t, path, h)
% -------------------------------------------------------------------
% Input:
% t: 查询时刻
% path: 一组航路点
% h: 用于可视化的句柄

% Output:
% s_des: 在查询时刻 t 的期望状态

% 函数说明:
% 该函数包含两部分，分别在 "if nargin > 1" 和 "else" 下实现。
% 第一部分根据输入的一组航路点生成轨迹。
% 第二部分在给定的查询时刻返回期望状态。
% -------------------------------------------------------------------

% 一些用于存储数据和生成轨迹系数的容器
persistent waypoints traj_time time_interval pos_coeffs vel_coeffs 
s_des = zeros(13,1);
s_des(7:10) = [1;0;0;0];  % 四元数，表示没有旋转

% Part I
if nargin > 1  % 生成轨迹 (给定航路点)
    % TODO: 
    % 假设四轴飞行器以单位速度飞行，计算它经过每个航路点的时间。
    
    % TODO: 最小快照轨迹 
    % 为7阶多项式准备 Q 矩阵
    Q = zeros(8 * num, 8 * num);
    
    % TODO: 
    % 准备映射矩阵 (多项式系数 --> 状态的导数)
    M = zeros(8 * num, 8 * num );
     
    % TODO:
	% 准备选择矩阵 C
    C = zeros(8 * num, 8 + 2 * (num - 1) + 3 * (num - 1));
    
    % TODO: 准备无约束二次规划 (QP) 问题的 R 矩阵
    R = 
    R_PP = 
    R_FP = 
    
    % TODO: 求解无约束 QP 问题
    % 准备 d_F
    d_Fx = zeros(2 * num + 6,1);
    d_Fy = zeros(2 * num + 6,1);
    d_Fz = zeros(2 * num + 6,1);
    
    % TODO: 求解 d_P
    d_Px_opt = 
    d_Py_opt = 
    d_Pz_opt = 
    
    % TODO: 将 d_F 和 d_P 叠加到 d 中
    d_x = 
    d_y = 
    d_z = 
    
    % TODO: 映射回多项式的系数 (位置)
    p_x = 
    p_y = 
    p_z = 
    
    pos_coeffs = 
    
    % TODO: 求解速度多项式的系数
    v_x = zeros(size(p_x));
    v_y = zeros(size(p_y));
    v_z = zeros(size(p_z));

    % ...
    
    vel_coeffs = [v_x v_y v_z];
    
    % 可视化: 绘制轨迹
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
    
% Part II: 返回在查询时刻的期望状态
else 
% 首先，检查是否超出范围？如果是，则调整为最终时刻。
    if t > traj_time(end)
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1) - 1;  
    
% 其次，处理 t == 0 的情况
    if t == 0
        s_des(1:3) = path(1,:);
        s_des(7:10) = [1;0;0;0];  % 四元数表示初始方向没有旋转
% 第三，处理正常情况
    else
        if t_index > 1
            t  = t - traj_time(t_index);
        end
       
        % 从生成的轨迹中加载系数
        px = pos_coeffs(:,1:8);
        py = pos_coeffs(:,9:16);
        pz = pos_coeffs(:,17:24);
        
        vx = vel_coeffs(:,1:8);
        vy = vel_coeffs(:,9:16);
        vz = vel_coeffs(:,17:24);
        
        % 计算在查询时刻的状态
        pos_x = polyval(flip(px(t_index,:)) , t);
        pos_y = polyval(flip(py(t_index,:)) , t);
        pos_z = polyval(flip(pz(t_index,:)) , t);
        
        vel_x = polyval(flip(vx(t_index,:)) , t);
        vel_y = polyval(flip(vy(t_index,:)) , t);
        vel_z = polyval(flip(vz(t_index,:)) , t);
        
        % 输出
        s_des(7:10) = [1;0;0;0];  % 始终输出初始方向四元数
        s_des(1) = pos_x;
        s_des(2) = pos_y;
        s_des(3) = pos_z;
        
        s_des(4) = vel_x;
        s_des(5) = vel_y;
        s_des(6) = vel_z; 
    end     
end

end
