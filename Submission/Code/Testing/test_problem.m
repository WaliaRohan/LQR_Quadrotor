clear; clc; close all;

x_i = [0; 0; 0; zeros(9, 1)];
x_d = [1; 1; 1; zeros(9, 1)];
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

figure(2)
plot(t, z_dot);
hold on;
plot(t, p);
plot(t, q);
plot(t, r);
legend('z_dot', 'p', 'q', 'r')
xlabel('t')
ylabel('State values')