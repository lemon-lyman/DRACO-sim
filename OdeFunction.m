function [Xdot] = OdeFunction(t, X, thrusterVec, P)
% quadOdeFunction : Ordinary differential equation function that models
%                   quadrotor dynamics.  For use with one of Matlab's ODE
%                   solvers (e.g., ode45).
%
%
% INPUTS
%
% t ---------- Scalar time input, as required by Matlab's ODE function
%              format.
%
% X ---------- Nx-by-1 quad state, arranged as 
%
%              X = [rI',vI',RBI(1,1),RBI(2,1),...,RBI(2,3),RBI(3,3),omegaB']'
%
%              rI = 3x1 position vector in I in meters
%              vI = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI = 3x3 attitude matrix from I to B frame
%          omegaB = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%              rI_debris = 3x1 position vector in I in meters
%              vI_debris = 3x1 velocity vector wrt I and in I, in meters/sec
%             RBI_debris = 3x3 attitude matrix from I to B frame
%          omegaB_debris = 3x1 angular rate vector of body wrt I, expressed in B
%                   in rad/sec
%
% thrusterVec
%
% P ---------- Structure with the following elements:
%
%    DRACOParams
%    DebrisParams
%
%    constants
%
% OUTPUTS
%
% Xdot ------- Nx-by-1 time derivative of the input vector X
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+

% Unpack state quantities
rI = X(1:3);
vI = X(4:6);
RBI = zeros(3,3);
RBI(:) = X(7:15);
omegaB = X(16:18);
omegaBx = crossProductEquivalent(omegaB);

rI_debris = X(19:21);
vI_debris = X(22:24);
RBI_debris = zeros(3,3);
RBI_debris(:) = X(25:33);
omegaB_debris = X(34:36);
omegaBx_debris = crossProductEquivalent(omegaB_debris);

% Assign some local variables for convenience
m_DRACO = P.DRACOParams.m;
m_debris = P.debrisParams.m;
Jq = P.DRACOParams.Jq;
Jq_debris = P.debrisParams.Jq;

Fvec = zeros(3, 1);

for ii=1:12
    % Force
    Fvec = Fvec + RBI*(P.DRACOParams.adcs_orientation(ii, :)')*thrusterVec(ii);
    % Torque
    NB = NB + cross(P.DRACOParams.adcs_position(ii, :),...
                    P.DRACOParams.adcs_orientation(ii, :)')*thrusterVec(ii);
end

point2point = (rI + RBI'*P.DRACOParams.attachmentPointB) - ...
              (rI_debris + RBI_debris'*P.debrisParams.attachmentPointB);

tetherForce = zeros(3, 1);
if (norm(point2point) > P.constants.minTetherLength)
    % create tether force
    tetherForce = P.constants.tetherK*(norm(point2point) - P.constants.minTetherLength)...
        *point2point*norm(point2point);
end

% Find derivatives of state elements
rIdot = vI;
vIdot = (RBI'*adcsFvec + tether_force)/m_DRACO;
RBIdot = -omegaBx*RBI;
omegaBdot = inv(Jq)*(NB - omegaBx*Jq*omegaB);

rI_debris_dot = vI_debris;
vI_debris_dot = (tether_force)/m_debris;
RBI_debris_dot = -omegaBx_debris*RBI_debris;
omegaB_debris_dot = inv(Jq_debris)*(NB_debris - omegaBx_debris*Jq_debris*omegaB_debris);

Xdot = [rIdot;
        vIdot;
        RBIdot(:);
        omegaBdot;
        rI_debris_dot;
        vI_debris_dot;
        RBI_debris_dot(:);
        omega_debris_Bdot];

end
