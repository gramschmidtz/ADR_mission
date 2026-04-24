clear; clc; close all;

% 1. 데이터 로드
load('trajectory_test.mat');

% 시간 변환 (초 -> 시간 단위)
t_hours = times / 3600;

% -----------------------------------------------------
% Figure 1: 3D 궤적 시각화
% -----------------------------------------------------
figure('Color', 'w', 'Name', 'Orbit 3D Trajectory');
hold on; grid on;

% 지구 그리기
R_e = 6378.14; % km
[x_sphere, y_sphere, z_sphere] = sphere(50);
surf(x_sphere*R_e, y_sphere*R_e, z_sphere*R_e, 'EdgeColor', 'none', 'FaceColor', '[0.2 0.5 0.8]', 'FaceAlpha', 0.4);

% 파편(Debris) 플롯 (파란색 실선)
mee_deb = squeeze(debris_states(:, 1, :));
[X_deb, Y_deb, Z_deb] = mee2cartesian(mee_deb);
plot3(X_deb/1e3, Y_deb/1e3, Z_deb/1e3, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Space Debris (No Thrust)');

% 위성(Chaser) 데이터가 있는지 확인하고 플롯 (빨간색 점선)
if exist('chaser_states', 'var')
    mee_cha = squeeze(chaser_states(:, 1, :));
    [X_cha, Y_cha, Z_cha] = mee2cartesian(mee_cha);
    plot3(X_cha/1e3, Y_cha/1e3, Z_cha/1e3, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Spacecraft (Thrust ON)');
end

xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('Spacecraft vs Debris Trajectories');
axis equal; view(3); legend('show');

% -----------------------------------------------------
% Figure 2: 제어 입력 (추력) 서브플롯
% -----------------------------------------------------
if exist('chaser_inputs', 'var')
    figure('Color', 'w', 'Name', 'Control Inputs');
    
    subplot(3,1,1);
    plot(t_hours, chaser_inputs(:, 1), 'r', 'LineWidth', 1.5);
    grid on; ylabel('N_r'); title('Control Inputs (Thrust Direction Vector)');
    ylim([-1.2 1.2]);
    
    subplot(3,1,2);
    plot(t_hours, chaser_inputs(:, 2), 'g', 'LineWidth', 1.5);
    grid on; ylabel('N_\theta');
    ylim([-1.2 1.2]);
    
    subplot(3,1,3);
    plot(t_hours, chaser_inputs(:, 3), 'b', 'LineWidth', 1.5);
    grid on; ylabel('N_h'); xlabel('Time (hours)');
    ylim([-1.2 1.2]);
end

% -----------------------------------------------------
% 헬퍼 함수: MEE -> Cartesian 변환
% -----------------------------------------------------
function [X, Y, Z] = mee2cartesian(mee_data)
    p = mee_data(:, 1);
    f = mee_data(:, 2);
    g = mee_data(:, 3);
    h = mee_data(:, 4);
    k = mee_data(:, 5);
    L = mee_data(:, 6);

    q = 1 + f .* cos(L) + g .* sin(L);
    r = p ./ q;
    s2 = 1 + h.^2 + k.^2;
    alpha2 = h.^2 - k.^2;

    X = (r ./ s2) .* (cos(L) + alpha2 .* cos(L) + 2 .* h .* k .* sin(L));
    Y = (r ./ s2) .* (sin(L) - alpha2 .* sin(L) + 2 .* h .* k .* cos(L));
    Z = (r ./ s2) .* (2 .* h .* sin(L) - 2 .* k .* cos(L));
end