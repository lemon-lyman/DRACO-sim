clc; clear; close all;

%% Define simulation parameters
% Total simulation time, in seconds
Tsim = 10.995;
% Control update interval, in seconds
delt = 0.005;
% Time vector, in seconds 
N = floor(Tsim/delt);
tVec=(0:N-1)'*delt;

%% Run relevant scripts
DRACOParamsScript;
ConstantsScript;
debrisParamsScript;

%% P - params
P.DRACOParams = DRACOParams; 
P.constants = constants; 
P.debrisParams = debrisParams;

%% S - state
S.tVec = tVec;

S.state0.rI = [0 0 0]';
S.state0.vI = [0 0 0]';
S.state0.e = [0 0 pi/2]';
S.state0.omegaB = [0 0 0]';

S.state0.rI_debris = [0 -10 0]';
S.state0.vI_debris = [0 0 0]';
S.state0.e_debris = [0 0 pi/2]';
S.state0.omegaB_debris = [0 0 0]';

S.thrusterMat = ; % Feed-forward thrusterVecs
S.oversampFact = 2;

%% Simulate
Q = simulateDRACO(S, P);

%% Visualize
visualizeSim(Q);