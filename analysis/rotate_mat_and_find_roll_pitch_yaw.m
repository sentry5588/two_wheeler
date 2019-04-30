%%
% Find roll, pitch, and yaw angles of the robot
% sentry5588
% April 2019
% Code runs on Octave

%%
clear all;
clc;

%% Calculate the sensor to robot coordinate transfer quaternion
% Actuall accelerometer reading in robot vertical position
% This is obtained by reading the raw values from MPU6050 when the robot
% is vertically stationary (e.g. sitting on the table)
a_sensor_0 = [16829.09091	85.09090909	-2744];
a_sensor_0 = a_sensor_0 / sqrt(a_sensor_0 * a_sensor_0'); % normalize

% Norminal robot "accelerometer" in robot vertical position
% This is the theoretical reading by assuming an accelerometer sensor is 
% mounted perfectly aligned with the robot
a_robot_0 = [0 0 -1];
a_robot_0 = a_robot_0 / sqrt(a_robot_0 * a_robot_0'); % normalize

q_s_r_a = 1 + a_robot_0 * a_sensor_0';
q_s_r_v = cross(a_robot_0, a_sensor_0);
q_s_r_temp = [q_s_r_a; q_s_r_v'];
q_s_r = q_s_r_temp / sqrt(q_s_r_temp' * q_s_r_temp)
q_s_r0 = q_s_r(1); q_s_r1 = q_s_r(2); q_s_r2 = q_s_r(3); q_s_r3 = q_s_r(4);

%%
% Simulate a sensor reading from MPU6050
a_sensor = [0 0 1];
a_sensor = a_sensor / sqrt(a_sensor * a_sensor'); % normalize

%% Transform MPU6050 acc. data from sensor coordinate to robot coordinate
a_robot = [0 0 1]; % Initialize the rotob Euler angle vector
a_robot(1) = (1 - 2 * q_s_r2^2 - 2 * q_s_r3^2) * a_sensor(1) ...
    + 2 * (q_s_r1 * q_s_r2 + q_s_r0 * q_s_r3) * a_sensor(2) ...
    + 2 * (q_s_r1 * q_s_r3 - q_s_r0 * q_s_r2) * a_sensor(3);
a_robot(2) = 2 * (q_s_r1 * q_s_r2 - q_s_r0 * q_s_r3) * a_sensor(1) ...
    + (1 - 2 * q_s_r1^2 - 2 * q_s_r3^2) * a_sensor(2) ...
    + 2 * (q_s_r2 * q_s_r3 + q_s_r0 * q_s_r1) * a_sensor(3);
a_robot(3) = 2 * (q_s_r1 * q_s_r3 + q_s_r0 * q_s_r2) * a_sensor(1) ...
    + 2 * (q_s_r2 * q_s_r3 - q_s_r0 * q_s_r1) * a_sensor(2) ...
    + (1 - 2 * q_s_r1^2 - 2 * q_s_r2^2) * a_sensor(3);
a_robot = a_robot / sqrt(a_robot * a_robot');

%% calculate robot rotation quaternion from vertical position
qr_a = 1 + a_robot * a_robot_0';
qr_v = cross(a_robot_0, a_robot);
qr_temp = [qr_a; qr_v'];
qr = qr_temp ./ sqrt(qr_temp' * qr_temp);
qr0 = qr(1); qr1 = qr(2); qr2 = qr(3); qr3 = qr(4);

% calculate the roll, pitch, yaw for sensor coordinate
roll = atan2(2*(qr2*qr3+qr0*qr1), 1-2*(qr1^2+qr2^2))
pitch = - asin(2*(qr1*qr3- qr0*qr2))
yaw = atan2(2*(qr1*qr2+qr0*qr3), 1-2*(qr2^2+qr3^2))

d = ['roll: ', ' (' num2str(roll/pi*180), ' Deg)' ...
    ' pitch: ', ' (',num2str(pitch/pi*180), ' Deg)'...
    ' yaw: ', ' (', num2str(yaw/pi*180), ' Deg)']

