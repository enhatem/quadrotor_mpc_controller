clear;
close all;
clc;

%% ACADO CODE

BEGIN_ACADO;

    acadoSet('problemname', 'full_dynamics_hovering');

    DifferentialState X Y Z Xd Yd Zd phi theta psi p q r;         % z and zDot

    Control T M1 M2 M3;                                           % Thrust

    Disturbance W;                                       % Disturbance on the 
                                                         % dynamics

    m = 0.027;                  % kg                     Static Parameters
    g = 9.81;                   % m/s^2
    Ixx = 2.3951e-5;            % kg.m^2
    Iyy = 2.3951e-5; 
    Izz = 3.2347e-5;

    km = 1.858e-5;              % N.m.s^2
    kf = 0.005022;              % N.s^2

    %% Differential equation

    f = acado.DifferentialEquation();                    % Set the differential equation object for OCP                                                     

    f.linkMatlabODE('ode');                              % Link a Matlab ODE
    
    
    %% Optimal Control Problem (OCP)

    ocp = acado.OCP(0.0, 1.0, 25);          % Set up the Optimal Control Problem (OCP)
                                            % Control in 25 intervals up to 1s

    h = {X, Y, Z, T};                             % The LSQ function

    Q = eye(4);                             % The weighting matrix
    Q(1,1) = 10;
    Q(2,2) = 10;
    Q(3,3) = 10;

    r = zeros(1,4);                         % The reference 
    
    
    ocp.minimizeLSQ( Q, h, r );             % Minimize the Least-Squares Term

    ocp.subjectTo( f );                     % Subject to the (simple) dynamics 

    ocp.subjectTo( 0 <= T <= 4*m*g );       % Bounds on the total thrust
    
    

    ocp.subjectTo( W == 0.0 );              % constraint for the disturbance


    %% SETTING UP THE (SIMULATED) PROCESS

    identity = acado.OutputFcn();

    dynamicSystem = acado.DynamicSystem(f, identity);       % Set up a dynamic system with the 
                                                            % the differential equation and
                                                            % an output function
    
    process = acado.Process(dynamicSystem, 'INT_RK45');     % Simulates the process to be 
                                                            % controlled based on 
                                                            % a dynamic model

    disturbance = [                                      % The process disturbance matrix consists of one
        0.0       0.00                                   % column with timepoints and one for each disturbance.
        0.5       0.00
        1.0       0.00
        1.49      0.00
        1.50      0.00
        3.0       0.00
        3.01      0.00
        4.0       0.00
        4.01      0.00
        5.0       0.00
        5.01      0.00
        6.0       0.00];
    % load('disturbance.mat');

    process.setProcessDisturbance(disturbance);         % Set the process disturbances

   % process.initializeAlgebraicStates([-0.3, m*g]);

    %% SETTING UP THE MPC CONTROLLER

    algo = acado.RealTimeAlgorithm(ocp, 0.02);         % The class RealTimeAlgorithm serves as a user-interface 
                                                       % to formulate and solve model predictive control problems.



    algo.set('MAX_NUM_ITERATIONS', 2 );                % Set some algorithm parameters

    %algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );
    
    algo.set('INTEGRATOR_TYPE', 'INT_RK45');
    algo.set( 'INTEGRATOR_TOLERANCE',   1e-5);    
    algo.set( 'ABSOLUTE_TOLERANCE',     1e-4 );
    algo.set( 'MAX_NUM_INTEGRATOR_STEPS',  1e6);
    % algo.set( 'MAX_STEPSIZE', 1e3);
    
    load ref.mat;
    
    reference = acado.PeriodicReferenceTrajectory(r);    % Allows to define a static reference trajectory that 
                                                       % the Control Law aims to track. 

    controller = acado.Controller(algo, reference);    % The controller complements a Process. 
                                                       % It contains an online control law for
                                                       % obtaining the control inputs of a process

    %% SETTING UP THE SIMULATION ENVIRONMENT

    sim = acado.SimulationEnvironment( 0.0, 6.0, process, controller ); % Setup the closed-loop simulation of the dynamic system. 
                                                                        % Simulate from 0 to 5 sec

    x0 = zeros(1,12);                                    % Initialize the states
    x0(1,3) = -0.3;

    sim.init( x0 );


END_ACADO;                                          % Generates hovering_RUN()

%% RUN THE SIMULATION

out = full_dynamics_hovering_RUN();

%draw;

