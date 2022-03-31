%{ 
This code represents the scnerio for which a simulated quadcopter is 
able to catch a foreign UAV in a given airspace of 10 x 10 x 10 units 
of volume. The quadcopter should start from the 'nest', which is situated
at the center of the airframe (0, 0, 0). If the quadcopter captures
the UAV, it should return to the nest.
%} 


tic % Code start time - to keep track of simulation time

clear; clc; close all;

% Sample trajectory for foreign UAV
x = [9 1 3 3];
y = [9 3 2 2];
z = [9 3 4 4.2];

%{
Note: Changing values in timeVector determines how fast the quadcopter
goes from one point to the next
%}
timeVector = [0 5 10 15 20]; 

%{
x_d is the state vector. Rows in x_d represent state quantities, and
columns represent state quantity value at each iteration.
x_dot = [x (x coordinate in world frame)
         y (y coordinate in world frame)
         z (z coordinate in world frame)
         psi (rotation about z-axis in body frame)
         theta (rotation about z-axis in body frame)
         phi (rotation about 
         r (rotation about ]
%}
x_d = [zeros(12, 1) [x; y; z; zeros(9, length(x))]];
original_path = x_d(:, 2:end);
trajectory = zeros(12, 1);

cap_x = -11; cap_y = -11; cap_z = -11; % capture point (set to out of
                                       % bounds as default)
                                       
isCaptured = false; % boolean flag for capture, set to false by default
                                       
for i=1:length(x_d(1,:))-1
    if(isOut(x_d(1, i+1), x_d(2, i+1), x_d(3, i+1)))
       x_d = [x_d(:, 1:i) zeros(12, 1)];
    end
    if (~isOut(x_d(1, i+1), x_d(2, i+1), x_d(3, i+1)))
       if(isNear(x_d(1:3, i), x_d(1:3, i+1)))
           cap_x = x_d(1, i); cap_y = x_d(2, i); cap_z = x_d(3, i);
           isCaptured = true;
           x_d = [x_d(:, 1:i) zeros(12, 1)];
           original_path = [original_path(:, 1:i) zeros(12, 1)]; 
       end
    end
    fprintf('%f | %f | %f \n', x_d(1, i), x_d(2, i), x_d(3, i));
    fprintf('%f | %f | %f \n', x_d(1, i+1), x_d(2, i+1), x_d(3, i+1));
    disp(norm([(x_d(1, i) - x_d(1, i+1)) (x_d(2, i) - x_d(2, i+1)) (x_d(2, i) - x_d(2, i+1))]));
    [t,X] = problem([timeVector(i):1:timeVector(i+1)], x_d(:, i), x_d(:, i+1));
    result = X(end, :)'; %grab final values of current iteration
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
    plot_des = plot3(original_path(1, :), original_path(2, :), original_path(3, :), 'b');
    grid on;
    hold on;
    plot_cov = plot3(trajectory(1, :), trajectory(2, :), trajectory(3, :), 'r');
    legend([plot_des plot_cov], 'Target Trajectory', 'System Trajectory');
    if (isCaptured)
       surface_plot = plot3(cap_x,cap_y, cap_z,'om','MarkerSize',40);
       legend([plot_des plot_cov surface_plot], 'Target Trajectory',...
           'System Trajectory', 'Capture Point');
    end

toc % code termination time - end of simulation
