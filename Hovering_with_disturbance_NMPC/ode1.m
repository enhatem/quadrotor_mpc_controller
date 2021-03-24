function [ dx ] = ode( t,x,u,p,w )
%ODE Represents the translational dynamics of a drone
%   

% Just for ease of use we write down all variables. This is not really needed. 
z     = x(1);  % vertical position
zd     = x(2); % vertical velocity
T      = u(1); % thrust
W      = w(1); % disturbance

% Constants

m = 0.027; 
g = 9.81;

% Differential equation

dx(1) = zd;
dx(2) = -T/m + g;


end

