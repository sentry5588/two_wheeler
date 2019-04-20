%%
% Find rotation matrix
% sentry5588
% April 2019
% Code runs on Octave
% Three tests were run at three robot positions:
% 1. Robot vertical [90 0 0] deg
% 2. Face up [0 90 0] deg
% 3. On the side [0 0 90] deg
% So the resulting matrix not only do rotation, but also do scaling

%%
clear all;
close all;
clc;

%%
v = 90; % deg
% robot matrix
robot_position = [v 0 0;
                0 v 0;
                0 0 v];
% raw data from the three tests [test1, test2, test3]
raw_data = [16829.33 265.8715 -782.713;
            -0.31663 -16401.5 331.2248;
            -2772.92 -1052.69 -17438.1];
			
% Since rotation/scaling matrix will do linear transform from
% raw data space to robot space
% so for test 1, the relation is:
% robot_position(:, 1) = R * raw_data(:, 1)
% so for test 2, the relation is:
% robot_position(:, 2) = R * raw_data(:, 2)
% so for test 3, the relation is:
% robot_position(:, 3) = R * raw_data(:, 3)
% therefore
% robot_position = R * raw_data
% hence R = robot_position * raw_data^-1

R = robot_position * inv(raw_data)