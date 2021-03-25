clear 
clc

syms phi theta psi p q r m L Ixx Iyy Izz g u v w F1 F2 F3 F4

syms x y z xDot yDot zDot phi theta psi p q r c

X = [ x y z xDot yDot zDot phi theta psi p q r].';

F = [ F1; F2; F3; F4];

%km= 1.858e-5;
%kf = 0.005022;

%c = km/kf;

M0 = zeros(3,3);

R_bi = [
    cos(theta)*cos(psi)                              cos(theta)*sin(psi)                            -sin(theta)
    sin(theta)*cos(psi)*sin(phi)-sin(psi)*cos(phi)  sin(theta)*sin(psi)*sin(phi)+cos(psi)*cos(phi)  cos(theta)*sin(phi)
    sin(theta)*cos(psi)*cos(phi)+sin(psi)*sin(phi)  sin(theta)*sin(psi)*cos(phi)-cos(psi)*sin(phi)  cos(theta)*cos(phi)];

A22 = [ 
    0       r/2     -q/2
    -r/2    0       p/2
    q/2     -p/2    0];

A23 = [
    0   -g*(1-(theta^2)/factorial(3)+(theta^4)/factorial(5))     0
    g*cos(theta)*(1-(phi^2)/factorial(3) + (phi^4)/factorial(5))    0   0
    g*((cos(theta)+1)/2)*(-phi/2+(phi^3)/factorial(4)-(phi^5)/factorial(6)) g*((cos(phi)+1)/2)*(-theta/2+(theta^3)/factorial(4)-(theta^5)/factorial(6))     0];

A24 = [
    0       -w/2    v/2
    w/2     0       -u/2
    -v/2    u/2     0];

A44 = [
    0                       ((Iyy-Izz)*r)/(2*Ixx)   ((Iyy-Izz)*q)/(2*Ixx)
    ((Izz-Ixx)*r)/(2*Iyy)   0                       ((Izz-Ixx)*p)/(2*Iyy)
    ((Ixx-Iyy)*q)/(2*Izz)   ((Ixx-Iyy)*p)/(2*Izz)   0                   ];

W = [
    1   sin(phi)*tan(theta)     cos(phi)*tan(theta)
    0   cos(phi)                -sin(phi)
    0   sin(phi)*sec(theta)     cos(phi)*sec(theta)];


Ac = [ 
    M0  R_bi.'  M0      M0
    M0  A22     A23     A24
    M0  M0      M0      W
    M0  M0      M0      A44];

M_0  = zeros(5,4);
M_0_ = zeros(3,4);

B_ = [
    0   -inv(Ixx)*L     0   inv(Ixx)*L
    inv(Iyy)*L  0   -inv(Iyy)*L     0
    -inv(Izz)*c     inv(Izz)*c  -inv(Izz)*c     inv(Izz)*c ];

B = [
    M_0
    -1/m*ones(1,4)
    M_0_
    B_];

XDot = Ac*X + B*F


