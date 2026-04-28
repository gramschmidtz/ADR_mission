%% 궤도 전파 애니메이션 시각화 스크립트 (MEE 변환 통합)
clear; clc; close all;

% 1. 데이터 로드
load('matlab/multi_debris_trajectory.mat'); 

% 지구 상수
R_e = 6378.14; % km

% 2. 피규어 설정
figure('Color', 'w', 'Position', [100, 100, 900, 700]);
hold on; grid on; axis equal;
view(135, 30);

% 축 범위 고정
limit_val = 10000;
axis([-limit_val limit_val -limit_val limit_val -limit_val limit_val]);

xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('Debris Orbit Propagation Animation');

% 3. 지구 그리기
[x_sphere, y_sphere, z_sphere] = sphere(50);
surf(x_sphere*R_e, y_sphere*R_e, z_sphere*R_e, 'EdgeColor', 'none', 'FaceColor', '[0.2 0.5 0.8]', 'FaceAlpha', 0.4);

% 4. 데이터 변환 (MEE -> Cartesian)
% Python에서 [Time, Batch, 7]로 넘어오므로 squeeze
mee_d1 = squeeze(debris0001_states(:, 1, :));
mee_d2 = squeeze(debris0002_states(:, 1, :));
mee_d3 = squeeze(debris0003_states(:, 1, :));

[X1, Y1, Z1] = mee2cartesian(mee_d1);
[X2, Y2, Z2] = mee2cartesian(mee_d2);
[X3, Y3, Z3] = mee2cartesian(mee_d3);

% 미터(m) 단위를 킬로미터(km)로 변환
X1 = X1 / 1e3; Y1 = Y1 / 1e3; Z1 = Z1 / 1e3;
X2 = X2 / 1e3; Y2 = Y2 / 1e3; Z2 = Z2 / 1e3;
X3 = X3 / 1e3; Y3 = Y3 / 1e3; Z3 = Z3 / 1e3;

% 5. 애니메이션용 객체 생성
colors = [1 0 0; 0 1 0; 0 0 1]; % Red, Green, Blue
names = {'Debris 0001', 'Debris 0002', 'Debris 0003'};
p_lines = gobjects(1, 3);
p_dots = gobjects(1, 3);

for i = 1:3
    p_lines(i) = plot3(nan, nan, nan, 'Color', colors(i,:), 'LineWidth', 1.5);
    p_dots(i) = plot3(nan, nan, nan, 'o', 'MarkerFaceColor', colors(i,:), 'MarkerSize', 6);
end
legend(p_dots, names, 'TextColor', 'black');

% 6. 애니메이션 루프
num_steps = length(times);
step_gap = 1; % 프레임 건너뛰기 간격

disp('애니메이션 시작...');
for k = 1:step_gap:num_steps
    % 궤적 선 업데이트
    set(p_lines(1), 'XData', X1(1:k), 'YData', Y1(1:k), 'ZData', Z1(1:k));
    set(p_lines(2), 'XData', X2(1:k), 'YData', Y2(1:k), 'ZData', Z2(1:k));
    set(p_lines(3), 'XData', X3(1:k), 'YData', Y3(1:k), 'ZData', Z3(1:k));
    
    % 현재 위치 점 업데이트
    set(p_dots(1), 'XData', X1(k), 'YData', Y1(k), 'ZData', Z1(k));
    set(p_dots(2), 'XData', X2(k), 'YData', Y2(k), 'ZData', Z2(k));
    set(p_dots(3), 'XData', X3(k), 'YData', Y3(k), 'ZData', Z3(k));
    
    drawnow;
end
disp('애니메이션 완료.');

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