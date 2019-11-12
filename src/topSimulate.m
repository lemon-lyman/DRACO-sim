


%%
R
%%
S
%%
P

%%
DRACOParamsScript;
constantsScript;
debrisParamsScript;
P.DRACOParams = DRACOParams; 
P.constants = constants; 
P.debrisParams = debrisParams;

%%
Q = simulateDRACOControl(R, S, P);

visualizeSim(Q);