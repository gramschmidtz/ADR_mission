% visualize_orbit.m
clear; clc; close all;

% 1. 데이터 로드
load('trajectory_test.mat');

% states 차원: [Time, Batch, 7]
% Batch 1번 데이터를 추출합니다.
mee_data = squeeze(states(:, 1, :)); 
t = times;

% MEE 변수 할당
p = mee_data(:, 1);
f = mee_data(:, 2);
g = mee_data(:, 3);
h = mee_data(:, 4);
k = mee_data(:, 5);
L = mee_data(:, 6);
mass = mee_data(:, 7);

% 2. MEE -> Cartesian (직교 좌표계) 변환
q = 1 + f .* cos(L) + g .* sin(L);
r = p ./ q;
s2 = 1 + h.^2 + k.^2;
alpha2 = h.^2 - k.^2;

X = (r ./ s2) .* (cos(L) + alpha2 .* cos(L) + 2 .* h .* k .* sin(L));
Y = (r ./ s2) .* (sin(L) - alpha2 .* sin(L) + 2 .* h .* k .* cos(L));
Z = (r ./ s2) .* (2 .* h .* sin(L) - 2 .* k .* cos(L));

% 3. 3D 궤적 플롯
figure('Color', 'w');
plot3(X/1e3, Y/1e3, Z/1e3, 'r-', 'LineWidth', 2);
hold on; grid on;

% 지구 그리기 (간단한 구 형태)
R_e = 6378.14; % km
[x_sphere, y_sphere, z_sphere] = sphere(50);
surf(x_sphere*R_e, y_sphere*R_e, z_sphere*R_e, 'EdgeColor', 'none', 'FaceColor', '[0.2 0.5 0.8]', 'FaceAlpha', 0.6);

xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');
title('Spacecraft Trajectory (MEE to Cartesian)');
axis equal;
view(3);