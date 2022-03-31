function [t,X] = problem(tspan, x0, xd)
    m = 0.2; g = 9.81;
    Ix = 1.24; Iy = 1.24; Iz = 2.48;

    
    e0  = xd - x0;
    
    
    f = @(x) [x(7);
        x(8);
        x(9);
        x(11)*(sin(x(6))/cos(x(5)))+x(12)*(cos(x(6))/cos(x(5)));
        x(11)*cos(x(6))-x(12)*sin(x(6));
        x(10)+x(11)*(sin(x(6))*tan(x(5)))+x(12)*(cos(x(6))*tan(x(5)));
        0;
        0;
        g;
        x(11)*x(12)*(Iy-Iz)/Ix;
        x(10)*x(12)*(Iz-Ix)/Iy;
        x(10)*x(11)*(Ix-Iy)/Iz];
    
    g17 = @(x) -(sin(x(6))*sin(x(4))+cos(x(6))*cos(x(4))*sin(x(5)))/m;
    g18 = @(x) -(cos(x(4))*sin(x(6))-cos(x(6))*sin(x(4))*sin(x(5)))/m;
    g19 = @(x) -(cos(x(6))*cos(x(5)))/m;
    g1 = @(x) [0; 0; 0; 0; 0; 0; g17(x); g18(x); g19(x); 0; 0; 0];
    g2 = [zeros(9,1); 1/Ix; 0; 0];
    g3 = [zeros(10,1); 1/Iy; 0];
    g4 = [zeros(11,1); 1/Iz];
    

    
    G = @(x) [g1(x)  g2  g3  g4];
    
A = [ 0, 0, 0,                                                  0,                                                     0,                                                   0, 1, 0, 0,                    0,                   0,                    0;
0, 0, 0,                                                  0,                                                     0,                                                   0, 0, 1, 0,                    0,                   0,                    0;
0, 0, 0,                                                  0,                                                     0,                                                   0, 0, 0, 1,                    0,                   0,                    0;
0, 0, 0,                                                  0, (sin(xd(5))*(xd(12)*cos(xd(6)) + xd(11)*sin(xd(6))))/cos(xd(5))^2,            (xd(11)*cos(xd(6)) - xd(12)*sin(xd(6)))/cos(xd(5)), 0, 0, 0,                    0,   sin(xd(6))/cos(xd(5)),    cos(xd(6))/cos(xd(5));
0, 0, 0,                                                  0,                                                     0,                     - xd(12)*cos(xd(6)) - xd(11)*sin(xd(6)), 0, 0, 0,                    0,            cos(xd(6)),            -sin(xd(6));
0, 0, 0,                                                  0,            (xd(12)*cos(xd(6)) + xd(11)*sin(xd(6)))/cos(xd(5))^2, (sin(xd(5))*(xd(11)*cos(xd(6)) - xd(12)*sin(xd(6))))/cos(xd(5)), 0, 0, 0,                    1,   sin(xd(6))*tan(xd(5)),    cos(xd(6))*tan(xd(5));
0, 0, 0, g*cos(xd(6))*sin(xd(4))*sin(xd(5)) - g*cos(xd(4))*sin(xd(6)),                         -g*cos(xd(4))*cos(xd(5))*cos(xd(6)),  g*cos(xd(4))*sin(xd(5))*sin(xd(6)) - g*cos(xd(6))*sin(xd(4)), 0, 0, 0,                    0,                   0,                    0;
0, 0, 0, g*(sin(xd(4))*sin(xd(6)) + cos(xd(4))*cos(xd(6))*sin(xd(5))),                          g*cos(xd(5))*cos(xd(6))*sin(xd(4)), -g*(cos(xd(4))*cos(xd(6)) + sin(xd(4))*sin(xd(5))*sin(xd(6))), 0, 0, 0,                    0,                   0,                    0;
0, 0, 0,                                                  0,                                   g*cos(xd(6))*sin(xd(5)),                                 g*cos(xd(5))*sin(xd(6)), 0, 0, 0,                    0,                   0,                    0;
0, 0, 0,                                                  0,                                                     0,                                                   0, 0, 0, 0,                    0, (xd(12)*(Iy - Iz))/Ix,  (xd(11)*(Iy - Iz))/Ix;
0, 0, 0,                                                  0,                                                     0,                                                   0, 0, 0, 0, -(xd(12)*(Ix - Iz))/Iy,                   0, -(xd(10)*(Ix - Iz))/Iy;
0, 0, 0,                                                  0,                                                     0,                                                   0, 0, 0, 0,  (xd(11)*(Ix - Iy))/Iz, (xd(10)*(Ix - Iy))/Iz,                    0];

 
B = [   0,     0,     0,     0;
   0,     0,     0,     0;
   0,     0,     0,     0;
   0,     0,     0,     0;
   0,     0,     0,     0;
   0,     0,     0,     0;
   0,     0,     0,     0;
   0,     0,     0,     0;
 1/m,     0,     0,     0;
   0, -1/Ix,     0,     0;
   0,     0, -1/Iy,     0;
   0,     0,     0, -1/Iz];

    %% Since x is a 12x1 matrix, taking R as a 12x12 identity matrix
    Q = 1000*eye(12); % for desired states
    %Q = diag([100, 100, 100, 10, ones(1, 8)]); % -> Tuning desired states

    %% Since u is a 4x1 matrix, taking R as a 4x4 identity matrix
    R = 0.01*eye(4); % for force
    %R = diag([10 10 10 10]); % -> Tuning force

    %% LQR function test to find optimal gain matrix K
    [K,~,~] = lqr(A,B,Q,R);

    %% Finding u as u = K*(x_d - x)
    u = @(e) -K*e + [m*g; 0 ; 0 ; 0];
 
    
    [t,e] =  ode45(@(t, e) -f(xd-e) - G(xd-e)*u(e), tspan, e0);
    
    X = zeros(size(e));
    
    for k = 1:length(t)
        X(k,:) = xd' - e(k,:);
    end
    
  
end





