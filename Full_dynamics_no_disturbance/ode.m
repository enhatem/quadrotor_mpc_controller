function [ dx ] = ode( t,x,u,p,w )
%ODE Represents the translational dynamics of a drone
%   

% Just for ease of use we write down all variables. This is not really needed. 
X      = x(1);
Y      = x(2);
Z      = x(3); 
Xd     = x(4); 
Yd     = x(5);
Zd     = x(6); 
phi    = x(7);
theta  = x(8);
psi    = x(9);
p      = x(10);
q      = x(11);
r      = x(12);

T      = u(1); % thrust
M1     = u(2); % Moment along x of the body-fixed frame
M2     = u(3); % Moment along y of the body-fixed frame
M3     = u(4); % Moment along z of the body-fixed frame


W      = w(1); % Disturbance

% Constants

m = 0.027;                  % kg
g = 9.81;                   % m/s^2
Ixx = 2.3951e-5;            % kg.m^2
Iyy = 2.3951e-5; 
Izz = 3.2347e-5;

%km = 1.858e-5;              % N.m.s^2
%kf = 0.005022;              % N.s^2


% Differential equation

dx(1) = Xd;
dx(2) = Yd;
dx(3) = Zd;
dx(4) = -(sin(psi)*sin(phi) + cos(psi)*sin(theta)*cos(phi))*T/m;
dx(5) = -(-cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi))*T/m;
dx(6) = -cos(theta)*cos(phi)*T/m + g;
dx(7) = p + q*(sin(phi)*tan(theta)) + r*(cos(phi)*tan(theta));
dx(8) =     q*cos(phi) - r*sin(phi);
dx(9) =     q*(sin(phi)*sec(theta)) + r*(cos(phi)*sec(theta));
dx(10) = ((Iyy - Izz)/Ixx)*q*r + M1/Ixx;
dx(11) = ((Izz - Ixx)/Iyy)*r*p + M2/Iyy;
dx(12) = ((Ixx - Iyy)/Izz)*p*q + M3/Izz;

end

