tic
clear; clc; close all;

duration = 11;
step = 1;

x = [0 1 3 4];
y = [0 3 2 5];
z = [0 3 4 2];

timeVector = [0 1 5 9 13];

x_d = [zeros(12, 1) [x; y; z; zeros(9, length(x))]];
trajectory = zeros(12, 1);

for i=1:length(x_d(1,:))-1
    disp("Start time: ")
    disp(timeVector(i))
    disp("End time: ")
    disp(timeVector(i+1))
    [t,X] = problem([timeVector(i):timeVector(i+1)], x_d(:, i), x_d(:, i+1));
    result = X'; %grab final values of current iteration
    trajectory = [trajectory result]; %store final values of current iteration
end 

figure(1)
for i=1:length(x_d(1, :))
    grid on;
    plot3(x_d(1, i), x_d(2, i), x_d(3, i), 'ok')
    hold on;
    pause(0.2)
end
for i=1:length(trajectory(1, :))
    grid on;
    plot3(trajectory(1, i), trajectory(2, i), trajectory(3, i), '.b')
    hold on;
    pause(0.001)
end
plot3(trajectory(1, :), trajectory(2, :), trajectory(3, :))
toc
