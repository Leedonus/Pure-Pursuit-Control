%% main.m
clc;
clear all;

%% Variables
lookaheadDistance = 0.25;
%ref_poses=struct2array(load('ref_poses.mat'));
Kp = 1;                                   %�ٶȿ�����ϵ��
k = 0;                                  %ǰ�Ӿ���ϵ��
dt = 0.1;                                 %״̬���¼��
WheelBase = 0.3;                          %���
Target_Speed = 1;
%%
cx = 0:0.1:10;
for i = 1:length(cx)                      %ȫ��·��c(y)���� ·����ʼ��
       cy(i) = cos(cx(i)/5)*cx(i)/2;
end
cx = cx';
cy = cy';
ref_poses = [cx,cy];
