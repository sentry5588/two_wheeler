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
rd = [16829.33 265.8715 -782.713;
            -0.31663 -16401.5 331.2248;
            -2772.92 -1052.69 -17438.1];
% Calculate vector length of each test
t1_length = sqrt(rd(:,1)' * rd(:,1))
t2_length = sqrt(rd(:,2)' * rd(:,2))
t3_length = sqrt(rd(:,3)' * rd(:,3))
max_t = max([t1_length, t2_length, t3_length]);
min_t = min([t1_length, t2_length, t3_length]);
mean_t = mean([t1_length, t2_length, t3_length])
direction_error = (max_t - min_t) / mean_t;

% check if tests are orthogonal
acosd(rd(:,1)' * rd(:,2) / (norm(rd(:,1)) * norm(rd(:,2))) );
acosd(rd(:,1)' * rd(:,3) / (norm(rd(:,1)) * norm(rd(:,3))) );
acosd(rd(:,2)' * rd(:,3) / (norm(rd(:,2)) * norm(rd(:,3))) );

% normalize the raw data to the same length
rd_normalized = [rd(:,1)./t1_length, ...
    rd(:,2)./t2_length, ...
    rd(:,3)./t3_length] * 90;

% An approximate method.
% A proper method is to be developped
% Since rotation/scaling matrix will do linear transform from
% raw data space to robot space
% so for test 1, the relation is:
% robot_position(:, 1) = R * rd(:, 1)
% so for test 2, the relation is:
% robot_position(:, 2) = R * rd(:, 2)
% so for test 3, the relation is:
% robot_position(:, 3) = R * rd(:, 3)
% therefore
% robot_position = R * rd
% hence R = robot_position * rd^-1

R = robot_position * inv(rd_normalized)
