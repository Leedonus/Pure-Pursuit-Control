%% initialization
k = 0.1;                                  % look forward gain  前向预测距离所用增益
Kp = 1.0 ;                                % speed propotional gain
Lfc = 1;                                  % 基础预瞄距离
dt = 0.1;                                 % [s]
L = 2.9  ;                                % [m] wheel base of vehicle
cx = 0:0.1:50;                            %每0.1长度取一次预瞄点的x值


for i = 1:length(cx)                      %全局路径c(y)生成 路径初始化
     cy(i) = cos(cx(i)/5)*cx(i)/4;
end



target_speed = 10/3.6;                    %设定速度  如果实际自动驾驶，这个就是设定要到达的速度，
                                          %附加：目前这里没有规划，如果有规划，就在过弯的时候给出横向加速度，所以车身方向速度就会减小。
T = 500;
lastIndex = length(cx);                   %返回cx的向量长度   有501个预瞄点x值   有501个序号
x = 0; y = 4; yaw = 0; v = 0;             %车辆初始状态  
time = 0;
Ld = k * v + Lfc;                         %初始速度为0时的预瞄距离Ld=Lfc
figure(1);
target_index = calc_target_index(x,y,v,cx,cy,k,Lfc);

while T > time && length(cx)>target_index
  target_index = calc_target_index(x,y,v,cx,cy,k,Lfc);                 %输出距离车辆当前最近的点的位置
  ai = PIDcontrol(target_speed,v,Kp); %
  Ld = k * v + Lfc ;
  delta = pure_pursuit_control(x,y,yaw,v,cx,cy,target_index,k,Lfc,L,Ld); %前轮要转的角度
   
  [x,y,yaw,v] = update(x,y,yaw,v, ai, delta,dt,L); %更新车辆状态
  time = time + dt;                          %时间过完一周期
% pause(0.1)
  plot(cx,cy,'g',x,y,'r-*')                 %将预瞄点位置用蓝色表示，当前位置用红色表示
  drawnow
  hold on
  
end


function [x, y, yaw, v] = update(x, y, yaw, v, ai, delta,dt,L)   %更新车辆状态的函数
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / L * tan(delta) * dt;
    v = v + ai * dt;
end

function [a] = PIDcontrol(target_v, current_v, Kp)              %计算加速度的函数
    a = Kp * (target_v - current_v);
end

function [delta] = pure_pursuit_control(x,y,yaw,v,cx,cy,ind,k,Lfc,L,Ld)  %纯追踪控制器前轮转角方程的函数
    tx = cx(ind);                          %距离车辆当前最近的点的位置对应的cx值     ind是距离车辆当前最近的点的位置
    ty = cy(ind);                          %距离车辆当前最近的点的位置对应的cy值
    alpha = atan((ty-y)/(tx-x))-yaw;       
    
    Ld = k * v + Lfc ;                        % 预瞄距离Ld与车速成线性关系 
    delta = atan(2*L * sin(alpha)/Ld)  ;                %前轮转角
end

function [ind] = calc_target_index(x,y,v, cx,cy,k,Lfc)      %计算找到距车辆当前位置最近点的序号
    N =  length(cx);                                     %N = 501
    Distance = zeros(N,1);                               % 501行 1列的矩阵

    for i = 1:N
    Distance(i) =  sqrt((cx(i)-x)^2 + (cy(i)-y)^2);      %每一个预瞄点到初始位置的距离
    end

    [~, location]= min(Distance);                        %找出最近的距离对应的位置
    ind = location;
    L=0;
    Lf = k*v+Lfc;
    while Lf>L && ind<length(cx)
        dx = cx(ind+1) - cx(ind);
        dy = cy(ind+1) - cy(ind);
        L = L + sqrt(dx^2+dy^2);
        ind= ind + 1;
    end
end