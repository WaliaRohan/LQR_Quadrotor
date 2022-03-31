clc; clear all; close all;

syms m g Ix Iy Iz 'positive'

syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 'real' 
syms x1d x2d x3d x4d x5d x6d x7d x8d x9d x10d x11d x12d 'real'   
syms e1 e2 e3 e4 e5 e6 e7 e8 e9 e10 e11 e12 'real' 


syms u1 u2 u3 u4 'real'
% x = [x; y; z; psi, theta, phi, xdot; ydot; zdot; p; q; r]
x = [x1; x2; x3; x4; x5; x6; x7; x8; x9; x10; x11; x12];
%xd = [x1d; x2d; x3d; x4d; x5d; x6d; x7d; x8d; x9d; x10d; x11d; x12d];
e = [e1; e2; e3; e4; e5; e6; e7; e8; e9; e10; e11; e12];

xd = [x1d; x2d; x3d; 0; 0; 0; x7d; x8d; x9d; 0; 0; 0];
u = [u1; u2; u3; u4];


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
    
    
%dx = @(x) f(x) + G(x)*u;

de =  @(e) -f(xd - e) - G(xd - e)*u;

A = jacobian(de(e),e);
A = subs(A, e, zeros(12,1));
A = subs(A, u, [m*g;0; 0; 0]);
A = simplify(A);
disp(A);

B = jacobian(de(e),u);
B = subs(B, e, zeros(12,1));
B = subs(B, u, [m*g;0; 0; 0]);
B = simplify(B);
disp(B);