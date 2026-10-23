%% NANENG 512: illustrative *pitch-only* inverted-pendulum model
% Not identified, not a full differential-drive robot model.
% Values below are examples for teaching only. MEASURE and REPLACE them.
% Control System Toolbox required for ss/c2d/dlqr.
% NOTE: Tiva firmware uses pitch PD/PID, NOT the LQR controller below.
clear; close all; clc;

Ts = 1/200;             % firmware's intended sample interval [s]
g = 9.81;              % gravitational acceleration [m/s^2]

% Replace each with measured / independently identified parameters:
m = 0.65;              % effective body mass [kg], PLACEHOLDER
l = 0.10;              % CoM height above axle [m], PLACEHOLDER
J = 0.012;             % effective pitch inertia [kg*m^2], PLACEHOLDER
b = 0.010;             % pitch viscous damping [N*m*s/rad], PLACEHOLDER
Ku = 0.20;             % actuator pitch-equivalent torque per u [N*m], PLACEHOLDER

% Small-angle linearization near upright: 
% J*theta_ddot = m*g*l*theta - b*theta_dot - Ku*u
% state x = [theta; theta_dot], pitch positive by chosen frame.
A = [0, 1; m*g*l/J, -b/J];
B = [0; -Ku/J];
C = eye(2);
D = zeros(2,1);
plant_c = ss(A, B, C, D);
plant_d = c2d(plant_c, Ts, 'zoh');

assert(rank(ctrb(plant_d.A, plant_d.B)) == 2, ...
    'Pitch-only model is uncontrollable with supplied parameters.');

% Illustrative discrete-time LQR weights; NOT actuator tuning values.
Q = diag([80, 4]);
R = 4;
K = dlqr(plant_d.A, plant_d.B, Q, R);
Acl = plant_d.A - plant_d.B*K;
disp('Illustrative discrete-time state feedback K:'); disp(K);
disp('Closed-loop discrete poles (should be inside unit circle):');
disp(eig(Acl));

% A simple linear closed-loop demonstration, no saturation nor disturbances.
time = (0:Ts:5)';
x = zeros(2, numel(time));
u = zeros(1, numel(time));
x(:,1) = [deg2rad(4); 0]; % initial pitch of 4 degrees
for k = 1:numel(time)-1
    u(k) = -K*x(:,k);
    x(:,k+1) = plant_d.A*x(:,k) + plant_d.B*u(k);
end
u(end) = -K*x(:,end);

figure('Name','Illustrative digital balance model');
subplot(2,1,1);
plot(time, rad2deg(x(1,:)), 'LineWidth',1.5); grid on;
ylabel('Pitch (deg)'); title('Nominal LQR response — not experimental');
subplot(2,1,2);
plot(time, u,'LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('Idealized u (not PWM-calibrated)');

% For an actual robot, extend state to [position, velocity, pitch, pitch_rate],
% identify motor + wheel parameters, add saturation and sensor/motor delays,
% calculate uncertainty, and validate the model against measured data.
