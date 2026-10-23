%% NANENG 512 - THEORETICAL PD vs open-loop pitch-only comparison
% Requires base MATLAB; does not use a real robot or measured data.
% Idealized model matches simulation/run_pitch_sim.py parameters.
% IMPORTANT: PD gains here are NOT the embedded firmware controller gains.
% Equation: J*theta_ddot = m*g*l*sin(theta)-b*theta_dot-Ku*u
% theta in rad; omega in rad/s; virtual PD gains in command per (deg, deg/s)
clear; close all; clc;

p.m = 0.65;       % kg: fictitious illustrative value
p.l = 0.10;       % m: fictitious illustrative value
p.J = 0.012;      % kg*m^2: fictitious illustrative value
p.b = 0.010;      % N*m*s/rad: fictitious illustrative value
p.Ku = 0.20;      % N*m/command: fictitious illustrative value
p.g = 9.81;       % m/s^2
p.kp = 0.09;      % norm command / degree, VIRTUAL PD ONLY
p.kd = 0.008;     % norm command / (degree/second), VIRTUAL PD ONLY
p.ulimit = 0.70;
p.Ts = 0.005;
p.tfinal = 8.0;
p.theta_max = 45.0; % arbitrary model validity stop, not MCU trip threshold

initial_theta_deg = 4.0;
[closed_t, closed_theta, closed_u] = simulate_case(p, initial_theta_deg, true);
[open_t, open_theta, open_u] = simulate_case(p, initial_theta_deg, false);

fprintf('*** THEORETICAL MODEL ONLY - NOT HARDWARE DATA ***\n');
fprintf('Controlled: final=%.4f deg, RMS=%.4f deg, peak=%.3f deg\n', ...
    closed_theta(end), sqrt(mean(closed_theta.^2)), ...
    max(abs(closed_theta)));
fprintf('Open loop: last=%.4f deg, terminated at %.3f s\n', ...
    open_theta(end), open_t(end));

figure('Name','THEORETICAL pitch-only robot (NOT measured)', 'Color','w');
subplot(2,1,1);
plot(closed_t, closed_theta, 'LineWidth',1.5); hold on;
plot(open_t, open_theta, '--', 'LineWidth',1.3);
yline(0, ':');
grid on; xlabel('Time (s)'); ylabel('Pitch (deg)');
legend('Illustrative virtual PD', 'Open loop', 'Reference 0 deg');
title('Simplified inverted pendulum — theoretical, not measured');
subplot(2,1,2);
plot(closed_t, closed_u,'LineWidth',1.5); hold on;
yline(p.ulimit, ':'); yline(-p.ulimit, ':');
grid on; xlabel('Time (s)'); ylabel('Ideal signed command');
title('Actuator command (not a measured PWM waveform)');
sgtitle('NANENG 512 THEORETICAL MODEL ONLY');

function [t, angle_deg, u_hist] = simulate_case(p, initial_deg, enabled)
    nsteps = round(p.tfinal/p.Ts);
    th = deg2rad(initial_deg);
    w = 0;
    t = zeros(nsteps+1, 1);
    angle_deg = zeros(nsteps+1, 1);
    u_hist = zeros(nsteps+1, 1);
    nfilled = 0;
    for k = 0:nsteps
        theta_deg = rad2deg(th);
        omega_deg_s = rad2deg(w);
        if enabled
            u = p.kp*theta_deg + p.kd*omega_deg_s;
            u = min(max(u, -p.ulimit), p.ulimit);
        else
            u = 0;
        end
        nfilled = nfilled + 1;
        t(nfilled) = k*p.Ts;
        angle_deg(nfilled) = theta_deg;
        u_hist(nfilled) = u;
        if abs(theta_deg) >= p.theta_max || k == nsteps
            break;
        end
        % One zero-order-held RK4 sample; u is held constant within the step.
        x = [th; w];
        dt = p.Ts;
        k1 = plant_rhs(x, u, p);
        k2 = plant_rhs(x + 0.5*dt*k1, u, p);
        k3 = plant_rhs(x + 0.5*dt*k2, u, p);
        k4 = plant_rhs(x + dt*k3, u, p);
        x = x + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
        th = x(1);
        w = x(2);
    end
    t = t(1:nfilled);
    angle_deg = angle_deg(1:nfilled);
    u_hist = u_hist(1:nfilled);
end

function dx = plant_rhs(x, u, p)
    dx = [x(2); (p.m*p.g*p.l*sin(x(1)) ...
        - p.b*x(2) - p.Ku*u)/p.J];
end
