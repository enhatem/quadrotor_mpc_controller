function [ dx ] = ode( t,x,u,p,dist )
%ODE Represents the translational dynamics of a drone
%   

% Just for ease of use we write down all variables. This is not really needed. 
X      =  x(1);
Y      =  x(2);
Z      =  x(3); 
u     =  x(4); 
v     =  x(5);
w     =  x(6); 
phi    =  x(7);
theta  =  x(8);
psi    =  x(9);
p      = x(10);
q      = x(11);
r      = x(12);

F1      = u(1); % thrust
F2     = u(2); % Moment along x of the body-fixed frame
F3     = u(3); % Moment along y of the body-fixed frame
F4     = u(4); % Moment along z of the body-fixed frame


W      = dist(1); % Disturbance

% Constants

m = 0.027;                  % kg
g = 9.81;                   % m/s^2
Ixx = 2.3951e-5;            % kg.m^2
Iyy = 2.3951e-5; 
Izz = 3.2347e-5;

km = 1.858e-5;              % N.m.s^2
kf = 0.005022;              % N.s^2

% Differential equation

% dx(1) = Xd;
% dx(2) = Yd;
% dx(3) = Zd;
% dx(4) = -(sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi))*T/m;
% dx(5) = -(-cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi))*T/m;
% dx(6) = -cos(theta)*cos(phi)*T/m + g;
% % dx(7) = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
% % dx(8) = q*cos(phi) - r*sin(phi);
% % dx(9) = (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta);
% dx(7) = p + q*(sin(phi)*tan(theta)) + r*(cos(phi)*tan(theta));
% dx(8) =     q*cos(phi) - r*sin(phi);
% dx(9) =     q*(sin(phi)*sec(theta)) + r*(cos(phi)*sec(theta));
% dx(10) = ((Iyy - Izz)/Ixx)*q*r + M1/Ixx;
% dx(11) = ((Izz - Ixx)/Iyy)*r*p + M2/Iyy;
% dx(12) = ((Ixx - Iyy)/Izz)*p*q + M3/Izz;

dx(1) = w*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - v*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + u*cos(psi)*cos(theta);
dx(2) = v*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - w*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + u*cos(theta)*sin(psi);
dx(3) = w*cos(phi)*cos(theta) - u*sin(theta) + v*cos(theta)*sin(phi);
dx(4) = (r*v)/2 - (q*w)/2 - (q*w)/2 + (r*v)/2 - g*theta*(theta^4/120 - theta^2/6 + 1);
dx(5) = (p*w)/2 - (r*u)/2 + (p*w)/2 - (r*u)/2 + g*phi*cos(theta)*(phi^4/120 - phi^2/6 + 1);
dx(6) = (q*u)/2 - (p*v)/2 - (p*v)/2 + (q*u)/2 - F1/m - F2/m - F3/m - F4/m - g*phi*(cos(theta)/2 + 1/2)*(phi^5/720 - phi^3/24 + phi/2) - g*theta*(cos(phi)/2 + 1/2)*(theta^5/720 - theta^3/24 + theta/2);
dx(7) = p + r*cos(phi)*tan(theta) + q*sin(phi)*tan(theta);
dx(8) = q*cos(phi) - r*sin(phi);
dx(9) = (r*cos(phi))/cos(theta) + (q*sin(phi))/cos(theta);
dx(10) = (F4*L)/Ixx - (F2*L)/Ixx + (q*r*(Iyy - Izz))/Ixx;
dx(11) = (F1*L)/Iyy - (F3*L)/Iyy - (p*r*(Ixx - Izz))/Iyy;
dx(12) = (F2*c)/Izz - (F1*c)/Izz - (F3*c)/Izz + (F4*c)/Izz + (p*q*(Ixx - Iyy))/Izz;

end

