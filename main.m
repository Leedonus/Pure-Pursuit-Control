%% main.m
clc;
clear all;

%% Variables
lookaheadDistance = 0.25;
%ref_poses=struct2array(load('ref_poses.mat'));
Kp = 1;                                   %速度控制器系数
k = 0;                                  %前视距离系数
dt = 0.1;                                 %状态更新间隔
WheelBase = 0.3;                          %轴距
Target_Speed = 1;
%%
cx = 0:0.1:10;
for i = 1:length(cx)                      %全局路径c(y)生成 路径初始化
       cy(i) = cos(cx(i)/5)*cx(i)/2;
end
cx = cx';
cy = cy';
ref_poses = [cx,cy];
