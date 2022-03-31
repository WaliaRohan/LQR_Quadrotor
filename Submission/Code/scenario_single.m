
clear; clc; close all;

x_i = zeros(12, 1);
x_d = [ones(3, 1); zeros(9, 1)];

[t,X] = problem([0 20], x_i, x_d);

disp("Final Values = ");
disp(X(:, end));

x = X(:, 1);
y = X(:, 2);
z = X(:, 3);

z_dot = X(:, 9);
p = X(:, 10);
q = X(:, 11);
r = X(:, 12);

figure(1)
plot(t, x);
hold on;
plot(t, y);
plot(t, z);
legend('x', 'y', 'z')
xlabel('t')
ylabel('State values')

disp('X Settling time: ')
disp(stepinfo(x, t).SettlingTime)
disp('Y Settling time: ')
disp(stepinfo(y, t).SettlingTime)
disp('Z Settling time: ')
disp(stepinfo(z, t).SettlingTime)

figure(2)
plot(t, z_dot);
hold on;
plot(t, p);
plot(t, q);
plot(t, r);
legend('z_dot', 'p', 'q', 'r')
xlabel('t')
ylabel('State values')

figure(3)
plot3(X(:, 1), X(:, 2), X(:, 3), 'b');
grid on;