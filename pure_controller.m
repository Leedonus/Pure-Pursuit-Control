%% initialization
k = 0.1;                                  % look forward gain  ǰ��Ԥ�������������
Kp = 1.0 ;                                % speed propotional gain
Lfc = 1;                                  % ����Ԥ�����
dt = 0.1;                                 % [s]
L = 2.9  ;                                % [m] wheel base of vehicle
cx = 0:0.1:50;                            %ÿ0.1����ȡһ��Ԥ����xֵ


for i = 1:length(cx)                      %ȫ��·��c(y)���� ·����ʼ��
     cy(i) = cos(cx(i)/5)*cx(i)/4;
end



target_speed = 10/3.6;                    %�趨�ٶ�  ���ʵ���Զ���ʻ����������趨Ҫ������ٶȣ�
                                          %���ӣ�Ŀǰ����û�й滮������й滮�����ڹ����ʱ�����������ٶȣ����Գ������ٶȾͻ��С��
T = 500;
lastIndex = length(cx);                   %����cx����������   ��501��Ԥ���xֵ   ��501�����
x = 0; y = 4; yaw = 0; v = 0;             %������ʼ״̬  
time = 0;
Ld = k * v + Lfc;                         %��ʼ�ٶ�Ϊ0ʱ��Ԥ�����Ld=Lfc
figure(1);
target_index = calc_target_index(x,y,v,cx,cy,k,Lfc);

while T > time && length(cx)>target_index
  target_index = calc_target_index(x,y,v,cx,cy,k,Lfc);                 %������복����ǰ����ĵ��λ��
  ai = PIDcontrol(target_speed,v,Kp); %
  Ld = k * v + Lfc ;
  delta = pure_pursuit_control(x,y,yaw,v,cx,cy,target_index,k,Lfc,L,Ld); %ǰ��Ҫת�ĽǶ�
   
  [x,y,yaw,v] = update(x,y,yaw,v, ai, delta,dt,L); %���³���״̬
  time = time + dt;                          %ʱ�����һ����
% pause(0.1)
  plot(cx,cy,'g',x,y,'r-*')                 %��Ԥ���λ������ɫ��ʾ����ǰλ���ú�ɫ��ʾ
  drawnow
  hold on
  
end


function [x, y, yaw, v] = update(x, y, yaw, v, ai, delta,dt,L)   %���³���״̬�ĺ���
    x = x + v * cos(yaw) * dt;
    y = y + v * sin(yaw) * dt;
    yaw = yaw + v / L * tan(delta) * dt;
    v = v + ai * dt;
end

function [a] = PIDcontrol(target_v, current_v, Kp)              %������ٶȵĺ���
    a = Kp * (target_v - current_v);
end

function [delta] = pure_pursuit_control(x,y,yaw,v,cx,cy,ind,k,Lfc,L,Ld)  %��׷�ٿ�����ǰ��ת�Ƿ��̵ĺ���
    tx = cx(ind);                          %���복����ǰ����ĵ��λ�ö�Ӧ��cxֵ     ind�Ǿ��복����ǰ����ĵ��λ��
    ty = cy(ind);                          %���복����ǰ����ĵ��λ�ö�Ӧ��cyֵ
    alpha = atan((ty-y)/(tx-x))-yaw;       
    
    Ld = k * v + Lfc ;                        % Ԥ�����Ld�복�ٳ����Թ�ϵ 
    delta = atan(2*L * sin(alpha)/Ld)  ;                %ǰ��ת��
end

function [ind] = calc_target_index(x,y,v, cx,cy,k,Lfc)      %�����ҵ��೵����ǰλ�����������
    N =  length(cx);                                     %N = 501
    Distance = zeros(N,1);                               % 501�� 1�еľ���

    for i = 1:N
    Distance(i) =  sqrt((cx(i)-x)^2 + (cy(i)-y)^2);      %ÿһ��Ԥ��㵽��ʼλ�õľ���
    end

    [~, location]= min(Distance);                        %�ҳ�����ľ����Ӧ��λ��
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