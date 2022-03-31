tic
clear; clc; close all;

duration = 11;
step = 1;

x = [1 3 11];
y = [3 2 5];
z = [3 4 2];

timeVector = [0 5 10 15];

x_d = [zeros(12, 1) [x; y; z; zeros(9, length(x))]];
original_path = x_d;
trajectory = zeros(12, 1);

for i=1:length(x_d(1,:))-1
    if(isOut(x_d(1, i+1), x_d(2, i+1), x_d(3, i+1)))
       x_d = [x_d(:, 1:i) zeros(12, 1)];
    end
    [t,X] = problem([timeVector(i):timeVector(i+1)], x_d(:, i), x_d(:, i+1));
    result = X'; %grab final values of current iteration
    trajectory = [trajectory result]; %store final values of current iteration
end 

figure(1)
for i=1:length(original_path(1, :))
    grid on;
    plot3(original_path(1, i), original_path(2, i), original_path(3, i), 'ok')
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
