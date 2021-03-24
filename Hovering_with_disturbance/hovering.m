clear;
close all;
clc;

%% ACADO CODE

BEGIN_ACADO;

    acadoSet('problemname', 'hovering');

    DifferentialState z zd;                              % z and zDot

    Control T;                                           % Thrust

    Disturbance W;                                       % Disturbance on the 
                                                         % dynamics

    m = 0.027;                                           % Static parameters
    g = 9.81;

    %% Differential equation

    f = acado.DifferentialEquation();                    % Set the differential equation object                                                     

    f.linkMatlabODE('ode');                              % Link a Matlab ODE

    %% Optimal Control Problem (OCP)

    ocp = acado.OCP(0.0, 1.0, 50);          % Set up the Optimal Control Problem (OCP)
                                            % Control in 50 intervals up to 1s

    h = {z, T};                             % The LSQ function

    Q = eye(2);                             % The weighting matrix
    Q(1,1) = 10;

    r = zeros(1,2);                         % The reference 
    r(1,1) = - 1;                           % set  reference for z to -1
    %r(1,2) = - 1;

    ocp.minimizeLSQ( Q, h, r );             % Minimize the Least-Squares Term

    ocp.subjectTo( f );                     % Subject to the dynamics

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
        1.50      3.00
        3.0       3.00
        3.01      0.00
        4.0       0.00
        4.01      -2.00
        5.0       -2.00
        5.01      0.00
        6.0       0.00];
    process.setProcessDisturbance(disturbance);         % Set the process disturbances

   % process.initializeAlgebraicStates([-0.3, m*g]);

    %% SETTING UP THE MPC CONTROLLER

    algo = acado.RealTimeAlgorithm(ocp, 0.02);         % The class RealTimeAlgorithm serves as a user-interface 
                                                       % to formulate and solve model predictive control problems.



    algo.set('MAX_NUM_ITERATIONS', 2 );                % Set some algorithm parameters

    % algo.set( 'HESSIAN_APPROXIMATION', 'GAUSS_NEWTON' );

    algo.set('INTEGRATOR_TYPE', 'INT_RK45');
    algo.set( 'INTEGRATOR_TOLERANCE',   1e-5);    
    algo.set( 'ABSOLUTE_TOLERANCE',     1e-4 );

    load ref.mat;
    
    reference = acado.PeriodicReferenceTrajectory(r);    % Allows to define a static reference trajectory that 
                                                       % the Control Law aims to track. 

    controller = acado.Controller(algo, reference);    % The controller complements a Process. 
                                                       % It contains an online control law for
                                                       % obtaining the control inputs of a process

    %% SETTING UP THE SIMULATION ENVIRONMENT

    sim = acado.SimulationEnvironment( 0.0, 6.0, process, controller ); % Setup the closed-loop simulation of the dynamic system. 
                                                                        % Simulate from 0 to 5 sec

    x0 = zeros(1,2);                                    % Initialize the states
    x0(1,1) = -0.3;
    %x0(1,2) = 0;

    sim.init( x0 );


END_ACADO;                                          % Generates hovering_RUN()

%% RUN THE SIMULATION

out = hovering_RUN();

draw;

