%% NANENG 512 — experimental data plotter (logger NOT yet implemented)
% Expected input: measured CSV with header
% time_s,angle_deg,gyro_x_dps,command
% Before real testing, add verified nonblocking UART logging to firmware.
clear; close all; clc;
file = 'measured_telemetry.csv';
if ~isfile(file)
    error(['CSV missing: ' file '. See script header for schema. ' ...
           'Do not present example/model traces as measured robot data.']);
end
T = readtable(file);
required = {'time_s','angle_deg','gyro_x_dps','command'};
assert(all(ismember(required, T.Properties.VariableNames)), ...
    'Required CSV columns: time_s, angle_deg, gyro_x_dps, command');
assert(all(diff(T.time_s) > 0), 'Timestamps must be strictly increasing.');
assert(all(isfinite(T.angle_deg)) && all(isfinite(T.command)), ...
    'Telemetry must have finite samples.');

figure('Name','Verified robot experimental telemetry');
subplot(3,1,1); plot(T.time_s, T.angle_deg); grid on;
ylabel('Angle (deg)'); title('Measured pitch / logged data');
subplot(3,1,2); plot(T.time_s, T.gyro_x_dps); grid on;
ylabel('Rate (deg/s)');
subplot(3,1,3); plot(T.time_s, T.command); grid on;
xlabel('Time (s)'); ylabel('Motor command');
fprintf('Records: %d, median sample interval: %.6f s\n', height(T), median(diff(T.time_s)));
