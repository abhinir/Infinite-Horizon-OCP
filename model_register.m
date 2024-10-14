function [model] = model_register(modelName)

%% pendulum
if strcmp(modelName, 'pendulum')
    model.name = 'pendulum';
    model.m = 0.5;
    model.L = 0.5;
    model.u_max = 1;
    model.dt = 0.01;
    model.nx = 2;
    model.nu = 1;
    model.g = 9.81;
    model.X0 = [60*pi/180;0];  %theta (rad), thetadot (rad/s)
    model.Xg = [0/180;0];
    model.Q = 100 * eye(model.nx) * model.dt;
    model.R = 10 * eye(model.nu) * model.dt;
    model.Qf = 1000*eye(model.nx);
    model.alpha = 1;
    % model around the equilibrium at the upright
    U_term = 0;
    [A,B] = pendulum_A_B(model, model.Xg, U_term);
    model.A = A;
    model.B = B;
    model.state_prop = @pendulum_nl_state_prop;
    model.cal_A_B = @pendulum_A_B;
    model.horizon = 3/model.dt;

elseif strcmp(modelName, 'car')
    model.name = 'car';
    model.L = 0.58; %length of the car
    model.u_max = 7;
    
    model.dt = 0.1;
    model.nx = 4; %[x,y,theta,velocity]
    model.nu = 2; %[acceleration,steering angle(rad)]
    model.alpha = 0.7;
    model.beta = 1; %discount factor.
    model.Cov = eye(model.nx); %noise covariance. 
    % lane change goal
    model.Xg = [10;5;0;0]; %[x,y,theta (rad),velocity]
    %model.Xg = [15;0;0;3];
    % model.X0 = [10.4626; 6.4612; 1.2644; -0.3284];% Case 2
    model.X0 = [6; -6;60*pi/180;10]; % Case 1
    model.R = 10*diag([1,100])*model.dt;%1*eye(model.nu);
    model.Q = 3*diag([1,1,0.0,0.01])*model.dt;
    model.Qf = 000*diag([1,1,0,0.01]);
    %[Ac, Bc] = cartpole_eqs(model);
    %model.Ac = Ac; % continuous time linearised model (symbolic)
    %model.Bc = Bc;
    model.nl_ode = @car_nl_ode;
    model.state_prop = @car_nl_state_prop;
    model.cal_A_B = @car_A_B;
    model.l = @car_state_cost;
    % model around the equilibrium at the upright
    U_term = [0;0];
    [A, B] = car_A_B(model, model.Xg, U_term);
    model.A = A;
    model.B = B;
    model.C = [1 0 0 0; 0 0 1 0];
    
    model.nz = 2; %number of outputs
    model.q = 1; %value of q required for ARMA model.
    model.nZ = model.q*model.nz + (model.q-1)*model.nu; %information-state dimension
    model.CC = [eye(model.nz), zeros(model.nz, model.nZ - model.nz)];%information state system C
    model.horizon = 500; %time horizon of the finite-horizon OCP
    model.nSim = 500;
    model.ptb = 0.0001;
    model.statePtb = 0.001;

elseif  strcmp(modelName, 'cartpole')
    model.name = 'cartpole';
    model.M = 1;
    model.m = 0.01;
    model.L = 0.6;
    model.u_max = 1;
    model.dt = 0.1;
    model.nx = 4;
    model.nu = 1;
    model.g = 9.81;
    model.alpha = 1;
    model.Xg = [0;0;0*pi/180;0]; %x, xdot, theta(rad), thetadot(rad/s)
    model.X0 = [0;0;180*pi/180;0];% pole bottom is pi
    model.R = 10*model.dt*eye(model.nu);
    model.Q = 100*model.dt*eye(model.nx);
    model.Qf = 1000*eye(model.nx);
    [Ac, Bc] = cartpole_eqs(model);
    model.Ac = Ac; % continuous time linearised model (symbolic)
    model.Bc = Bc;
    % model around the equilibrium at the upright
    U_term = 0;
    [A, B] = cartpole_A_B(model, model.Xg, U_term);
    model.A = A;
    model.B = B;
    model.state_prop = @cartpole_nl_state_prop;
    model.cal_A_B = @cartpole_A_B;
    model.C = [1 0 0 0; 0 0 1 0];
    
    model.nz = 2; %number of outputs
    model.q = 2; %value of q required for ARMA model.
    model.nZ = model.q*model.nz + (model.q-1)*model.nu; %information-state dimension
    model.CC = [eye(model.nz), zeros(model.nz, model.nZ - model.nz)];%information state system C
    model.horizon = 3/model.dt; %time horizon of the finite-horizon OCP
    model.nSim = 500;
    model.ptb = 0.0001;
    model.statePtb = 0.001;

elseif strcmp(modelName, '1dcos')
    
    model.name = '1dcos';
    model.u_max = 1;
    model.nx = 1;
    model.nu = 1;
    model.alpha = 1;
    model.Xg = 0;
    model.X0 = 1;
    model.state_prop = @cos1d_state_prop;
    model.cal_A_B = @cos1d_A_B;
    model.horizon = 200;
    model.dt = 1.0/model.horizon;
    model.R = 1*model.dt;
    model.Q = 100*model.dt;
    model.Qf = 500;
    
elseif  strcmp(modelName, 'softLand')
    model.name = 'softLand';
    model.m = 2000;
    model.J = diag([4500,2000,7500]);
    model.r1 = [-2;0;1];
    model.r2 = [2;0;1];
    model.r3 = [0;-2;1];
    model.r4 = [0;2;1];
    model.d1 = [0.7071;0;0.7071];
    model.d2 = [-0.7071;0;0.7071];
    model.d3 = [0;0.7071;0.7071];
    model.d4 = [0;-0.7071;0.7071];
    
    model.dt = 0.2;
    model.nx = 13;
    model.nu = 6;
    model.g = [0;0;-3.7114];
    model.alpha = 1;
    model.Xg = [0;0;0;0;0;0;0;0;0;0;0;0;1500]; 
    model.X0 = [0.4;0.3;0.2;0.1;0.2;-0.2;0.03;-0.02;0.1;0.1;0.12;0;2000];
    % model.X0 = [0;0;0;0;0;0;0.001000;-0.002000;0.001000;0.05;-0.06;0.050;500];
    model.R = 5*diag([1,1,1,1000,1000,1000])*model.dt;
    model.Q = diag([1e3,1e3,1e3,1e3,1e3,1e3,5e8,5e8,5e6,1e6,1e6,2e6,0])*model.dt;
    model.Qf = 10e1*diag([1e6,1e6,1e6,1e6,1e6,1e6,10e8,10e8,10e8,5e8,5e8,5e8,0]);
    model.l = @softLand_state_cost;
    % [Ac, Bc] = softLand_eqs(model);
    % model.Ac = Ac; % continuous time linearised model (symbolic)
    % model.Bc = Bc;
    % model around the equilibrium at the upright
    model.U_term = [zeros(5,1);0.1500*3.7114];
    [A, B] = softLand_A_B(model, model.Xg, model.U_term);
    model.A = A;
    model.B = B;
    model.state_prop = @softLand_nl_state_prop;
    model.cal_A_B = @softLand_A_B;
    model.C = [1 0 0 0; 0 0 1 0];
    
    model.nz = 2; %number of outputs
    model.q = 0; %value of q required for ARMA model.
    model.nZ = model.q*model.nz + (model.q-1)*model.nu; %information-state dimension
    model.CC = [eye(model.nz), zeros(model.nz, model.nZ - model.nz)];%information state system C
    model.horizon = 70/model.dt; %time horizon of the finite-horizon OCP
    model.nSim = 500;
    model.ptb = 0.0001;
    model.statePtb = 0.001;
elseif  strcmp(modelName, 'rendezvous')
    model.name = 'rendezvous';

    model.dt = 2;
    model.nx = 13;
    model.nu = 3;
    model.mu = 398600.4418;
    model.alpha = 1;
    
    model.r10 = [-6296.30;817.09;3591.96];
    model.v10 = [-3.05;-6.00;-2.65];
    model.r20 = [-2192.51;-6164.08;-1033.77];
    model.v20 = [3.12;-1.68;-7.24];

    model.X0 = [model.r10 - model.r20;model.v10 - model.v20;2000;model.r10;model.v10];% pole bottom is pi
    model.Xg = [zeros(6,1);1000;model.r10;model.v10];

    model.R = 1000*eye(model.nu)*model.dt;
    model.Q = diag([1,1,1,1e5,1e5,1e5,0,0,0,0,0,0,0])*model.dt;
    model.Qf = 1e-2*diag([1e9,1e9,1e9,1e9,1e9,1e9,0,0,0,0,0,0,0])*model.dt;
    % [Ac, Bc] = softLand_eqs(model);
    % model.Ac = Ac; % continuous time linearised model (symbolic)
    % model.Bc = Bc;
    % model around the equilibrium at the upright
    model.U_term = zeros(3,1);
    [A, B] = rendezvous_A_B(model, model.Xg, model.U_term);
    model.A = A;
    model.B = B;
    model.state_prop = @rendezvous_nl_state_prop;
    model.cal_A_B = @rendezvous_A_B;
    model.C = [1 0 0 0; 0 0 1 0];
    
    model.nz = 2; %number of outputs
    model.q = 2; %value of q required for ARMA model.
    model.nZ = model.q*model.nz + (model.q-1)*model.nu; %information-state dimension
    model.CC = [eye(model.nz), zeros(model.nz, model.nZ - model.nz)];%information state system C
    model.horizon = 2000/model.dt; %time horizon of the finite-horizon OCP
    model.nSim = 500;
    model.ptb = 0.0001;
    model.statePtb = 0.001;
elseif strcmp(modelName,'att_con')
    model.name = 'att_con';
    model.m = 2000;
    model.J = diag([4500,2000,7500]);
    model.dt = 0.1;
    model.nx = 6;
    model.nu = 3;
    model.alpha = 1;
    model.Xg = [0;0;0;0;0;0];
    model.X0 = [1.5;-1.2;-2.1;0.1;-0.1;0.05];
    model.R = 10*eye(model.nu)*model.dt;
    model.Q = 1e4*eye(model.nx)*model.dt;
    model.Qf = 1e6*eye(model.nx);
    model.state_prop = @att_con_nl_state_prop;
    model.cal_A_B = @att_con_A_B;
    model.horizon = 300/model.dt;
    model.U_term = zeros(3,1);
    [A, B] = att_con_A_B(model, model.Xg, model.U_term);
    model.A = A;
    model.B = B;
elseif strcmp(modelName,'double_integrator')
    model.name = 'double_integrator';
    model.dt = 0.1;
    model.nx = 2;
    model.nu = 1;
    model.R = 10*eye(model.nu)*model.dt;
    model.Q = 0*eye(model.nx)*model.dt;
    model.Qf = 1e5*eye(model.nx);
    model.Xg = [0;0];
    model.X0 = [1.5;-1.2];
    model.cal_A_B = @double_integrator_A_B;
    model.horizon = 3/model.dt;
    model.alpha = 1;
    model.state_prop = @double_integrator_state_prop;

elseif strcmp(modelName,'toy_rao')
    model.name = 'toy_rao';
    model.dt = 0.1;
    model.nx = 1;
    model.nu = 1;
    model.R = 20*eye(model.nu)*model.dt;
    model.Q = 20*eye(model.nx)*model.dt;
    model.Qf = 1e10*eye(model.nx);
    model.Xg = 1.5;
    model.X0 = 1;
    model.cal_A_B = @toy_rao_A_B;
    model.horizon = 25/model.dt;
    model.alpha = 1;
    model.state_prop = @toy_rao_state_prop;
end

