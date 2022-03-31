clear; clc; close all;

syms x y z psi theta phi xdot ydot zdot p q r;
x_sym = [x; y; z; psi; theta; phi; xdot; ydot; zdot; p; q; r]; 
x_num_i = 10*ones(12, 1);
split = 3;
x_num_d = [ones(split, 1); zeros(12-split, 1)];
f_sym(x_num_i, x_num_d);


x_i = [1; 1; 1; zeros(9, 1)];
x_d = [0; 0; 0; zeros(9, 1)];
result = f([0 3], x_i, x_d);

disp("Final Values - ");
disp(result.y(:, end));

time = result.x;

x = result.y(1, :);
y = result.y(2, :);
z = result.y(3, :);

z_dot = result.y(9, :);
p = result.y(10, :);
q = result.y(11, :);
r = result.y(12, :);

figure(1)
plot(time, x);
hold on;
plot(time, y);
plot(time, z);
legend('x', 'y', 'z')
xlabel('Time')
ylabel('State values')

figure(2)
plot(time, z_dot);
hold on;
plot(time, p);
plot(time, q);
plot(time, r);
legend('z_dot', 'p', 'q', 'r')
xlabel('Time')
ylabel('State values')