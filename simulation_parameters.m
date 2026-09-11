function cfg = simulation_parameters(useAdaptiveTuning)
%SIMULATION_PARAMETERS Numerical parameters used in both simulations.

if nargin < 1
    useAdaptiveTuning = true;
end

cfg.useAdaptiveTuning = logical(useAdaptiveTuning);
cfg.useDisturbance = true;
cfg.trainDDPGonline = false;

% Robot geometry
cfg.L1 = 1.2;
cfg.L2 = 1.0;
cfg.L3 = 0.2;
cfg.cubeSize = 0.1;
cfg.baseL = [-0.5; 0];
cfg.baseR = [ 0.5; 0];

% Link and object parameters
cfg.m1 = 1;
cfg.m2 = 1;
cfg.m3 = 1;
cfg.lc1 = cfg.L1/2;
cfg.lc2 = cfg.L2/2;
cfg.lc3 = cfg.L3/2;
cfg.mObj = 0.8;
cfg.g = 9.81;
cfg.mu = 0.15;
cfg.W = cfg.mObj * cfg.g;

% Simulation time
cfg.T = 5;
cfg.dt = 0.02;
cfg.t = 0:cfg.dt:cfg.T;
cfg.N = numel(cfg.t);

% MPC horizon and fixed weights
cfg.H = 15;
cfg.QpDefault = 100 * eye(2);
cfg.QphiDefault = 45;
cfg.RDefault = 6e-2 * eye(3);
cfg.SDefault = 1e-3 * eye(2);
cfg.RduDefault = 5e-2 * eye(3);

% State, input and force limits
cfg.qmin = deg2rad([-170; -170; -170]);
cfg.qmax = deg2rad([ 170;  170;  170]);
cfg.umax = deg2rad([150; 150; 150]);
cfg.umin = -cfg.umax;
cfg.fnMax = 200;
cfg.etaRef = 0.55;

% Adaptive weight ranges
cfg.rhoMin = [50; 20; 1e-4; 1e-4];
cfg.rhoMax = [1200; 800; 5e-1; 5e-1];
cfg.alphaRho = 0.95;
cfg.rho0 = [cfg.QpDefault(1,1); cfg.QphiDefault; ...
            cfg.RDefault(1,1); cfg.SDefault(1,1)];

% Disturbed plant used for the comparison
cfg.actGainL = [0.82; 1.15; 0.88];
cfg.actGainR = [1.12; 0.85; 1.18];
cfg.distAmp = deg2rad(10);
cfg.noiseAmp = deg2rad(3);
cfg.randomSeed = 10;

cfg.optimizer = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','none', ...
    'MaxIterations',200, ...
    'OptimalityTolerance',1e-5, ...
    'StepTolerance',1e-7);
end
