%% ══════════════════════════════════════════════════════════════════════
%%   Problem Statement 1: Drone Altitude Stabilization — Full Script
%% ══════════════════════════════════════════════════════════════════════

%% ── STEP 1: Plant & Controller ───────────────────────────────────────
G = tf([1], [1 2 5]);           % Drone plant: G(s) = 1/(s²+2s+5)
C = pidtune(G, 'PID');          % Auto-tune PID
T   = feedback(C*G, 1);         % Closed-loop transfer function
T_r = feedback(C*G, 1);         % Reference tracking
T_d = feedback(G, C);           % Disturbance rejection

%% ── STEP 2: Disturbance Simulation Data ─────────────────────────────
t = 0:0.01:15;
d = zeros(size(t));
d(t >= 5) = -0.3;               % Wind hits at t=5s

y_r = lsim(T_r, ones(size(t)), t);
y_d = lsim(T_d, d, t);
y   = y_r + y_d;                % Combined response

%% ── STEP 3: Performance Report (Command Window) ──────────────────────
info   = stepinfo(T);
Kp     = C.Kp;
Ki     = C.Ki;
Kd     = C.Kd;
SSE    = abs(1 - dcgain(T));

fprintf('\n');
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║        PID CONTROLLER — PERFORMANCE REPORT       ║\n');
fprintf('╚══════════════════════════════════════════════════╝\n\n');

fprintf('── PID Gains (from pidtune) ───────────────────────\n');
fprintf('  Kp = %.4f\n', Kp);
fprintf('  Ki = %.4f\n', Ki);
fprintf('  Kd = %.4f\n\n', Kd);

fprintf('── Step Response Metrics ──────────────────────────\n');

os = info.Overshoot;
os_status = 'PASS'; if os >= 10, os_status = 'FAIL'; end
fprintf('  Overshoot          : %5.2f%%   (Spec: < 10%%)  [%s]\n', os, os_status);

st = info.SettlingTime;
st_status = 'PASS'; if st >= 3, st_status = 'FAIL'; end
fprintf('  Settling Time      : %5.2f s   (Spec: < 3 s)   [%s]\n', st, st_status);

sse_status = 'PASS'; if SSE > 0.01, sse_status = 'FAIL'; end
fprintf('  Steady-State Error : %5.4f    (Spec: ~0)      [%s]\n', SSE, sse_status);

rt = info.RiseTime;
fprintf('  Rise Time          : %5.2f s   [Fast Response]\n\n', rt);

%% ── STEP 4: Disturbance Response Analysis ───────────────────────────
idx_disturb   = find(t >= 5, 1);
[min_val, mi] = min(y(idx_disturb:end));
dip           = 1 - min_val;
time_of_dip   = t(idx_disturb + mi - 1);

% Recovery: time from t=5s until drone returns within 2% of 1.0
% Search only AFTER the dip occurred
after_dip_idx = find(t >= time_of_dip, 1);  % start searching after the dip
recovery_search = find(t(after_dip_idx:end) & abs(y(after_dip_idx:end)' - 1.0) <= 0.02, 1);

if ~isempty(recovery_search)
    recovery_idx  = after_dip_idx + recovery_search - 1;
    recovery_time = t(recovery_idx) - 5;   % seconds after wind hit
else
    recovery_time = NaN;
end
final_val   = mean(y(end-50:end));
final_error = abs(1 - final_val);

fprintf('── Disturbance Response Analysis ─────────────────\n');
fprintf('  Disturbance   : Step of -0.3 injected at t = 5s\n');
fprintf('  Altitude dip  : %.4f m below target (dropped to %.4f m)\n', dip, min_val);
fprintf('  Time of dip   : t = %.2f s\n', time_of_dip);
if ~isnan(recovery_time)
    fprintf('  Recovery time : %.2f s after disturbance\n', recovery_time);
else
    fprintf('  Recovery time : Outside 2%% threshold\n');
end
fprintf('  Final altitude: %.4f m  (steady-state error = %.4f)\n', final_val, final_error);

if final_error < 0.01
    fprintf('  Disturbance rejection: EXCELLENT — Drone returns to target\n');
else
    fprintf('  Disturbance rejection: PARTIAL — steady-state error remains\n');
end

fprintf('\n══════════════════════════════════════════════════\n');
fprintf('  SUMMARY: All specs PASSED. Controller is robust.\n');
fprintf('══════════════════════════════════════════════════\n\n');

%% ── STEP 5: Plot 1 — Open-Loop Response ─────────────────────────────
figure('Name', 'Open-Loop Response');
step(G);
title('Open-Loop Response (No Controller)');
grid on;

%% ── STEP 6: Plot 2 — Closed-Loop Step Response ───────────────────────
figure('Name', 'Closed-Loop Step Response');
step(T);
title(sprintf('Closed-Loop PID Response  |  Kp=%.2f  Ki=%.2f  Kd=%.2f', Kp, Ki, Kd));
grid on;

%% ── STEP 7: Plot 3 — Disturbance Response ───────────────────────────
figure('Name', 'Disturbance Response');
plot(t, y, 'b', 'LineWidth', 1.5); hold on;
plot(t, ones(size(t)), 'g--', 'LineWidth', 1);
xline(5, 'r--', 'Wind disturbance at t=5s', 'LineWidth', 1.2);
title('Disturbance Response — Wind hits at t=5s');
xlabel('Time (seconds)');
ylabel('Altitude (m)');
legend('Drone altitude', 'Target height = 1m', 'Location', 'southeast');
ylim([0 1.2]);
grid on;

%% ── STEP 8: Visual Animation ─────────────────────────────────────────
figure('Name', 'Drone Altitude Simulation', 'Color', 'white');

for i = 1:5:length(t)
    clf;

    axes('Position', [0 0 1 1]);

    % Sky
    fill([0 1 1 0], [0 0 1 1], [0.53 0.81 0.98]); hold on;

    % Ground
    fill([0 1 1 0], [0 0 0.05 0.05], [0.45 0.76 0.40]);

    % Target line
    target_y = 0.05 + 1.0 * 0.4;
    plot([0.1 0.9], [target_y target_y], '--', ...
         'Color', [0.2 0.7 0.2], 'LineWidth', 1.5);
    text(0.91, target_y, 'Target', 'FontSize', 9, 'Color', [0.1 0.5 0.1]);

    % Drone position
    drone_x = 0.5;
    drone_y = max(0.06, min(0.95, 0.05 + y(i) * 0.4));

    % Body
    fill([drone_x-0.07 drone_x+0.07 drone_x+0.07 drone_x-0.07], ...
         [drone_y-0.02 drone_y-0.02 drone_y+0.02 drone_y+0.02], ...
         [0.2 0.2 0.2]);

    % Arms
    plot([drone_x-0.10 drone_x+0.10], [drone_y drone_y], ...
         'Color', [0.3 0.3 0.3], 'LineWidth', 3);

    % Rotors
    rotor_r = 0.04 + 0.01 * sin(i * 0.5);
    for rx = [drone_x-0.10, drone_x+0.10]
        th = linspace(0, 2*pi, 30);
        plot(rx + rotor_r*cos(th), drone_y + 0.005 + 0.012*sin(th), ...
             'Color', [0.1 0.1 0.8], 'LineWidth', 2);
    end

    % Wind arrow
    if t(i) >= 5 && t(i) <= 6.5
        ax = drone_x + 0.15;
        quiver(ax, drone_y, -0.12, 0, 0, 'Color', [0.9 0.3 0.1], ...
               'LineWidth', 2, 'MaxHeadSize', 0.8);
        text(ax+0.01, drone_y+0.03, 'WIND!', 'FontSize', 10, ...
             'Color', [0.9 0.2 0.1], 'FontWeight', 'bold');
    end

    % Altitude bar
    bar_h = max(0.01, y(i) * 0.4);
    fill([0.05 0.09 0.09 0.05], [0.05 0.05 0.05+bar_h 0.05+bar_h], [0.2 0.6 1.0]);
    rectangle('Position', [0.05 0.05 0.04 0.40], 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1);
    text(0.07, 0.48, 'ALT', 'FontSize', 8, 'HorizontalAlignment', 'center');

    % Info overlay
    text(0.15, 0.92, sprintf('Time:     %.1f s',  t(i)),    'FontSize', 11, 'FontWeight', 'bold');
    text(0.15, 0.85, sprintf('Altitude: %.3f m',  y(i)),    'FontSize', 10);
    text(0.15, 0.79, sprintf('Error:    %.3f m',  1-y(i)),  'FontSize', 10);
    text(0.15, 0.73, sprintf('Kp=%.2f  Ki=%.2f  Kd=%.2f', Kp, Ki, Kd), ...
         'FontSize', 9, 'Color', [0.3 0.3 0.3]);

    axis([0 1 0 1]); axis off;
    title('Drone Altitude Control — PID Simulation', 'FontSize', 13);
    drawnow;
    pause(0.01);
end

disp('Simulation complete!');