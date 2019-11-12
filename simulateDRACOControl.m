function [Q] = simulateDRACOControl(R, S, P)
% simulateQuadrotorControl : Simulates closed-loop control of a quadrotor
%                            aircraft.
%
%
% INPUTS
%
% R ---------- Structure with the following elements:
%
%          tVec
%        rIstar
%        vIstar
%        aIstar
%        xIstar
%  
% S ---------- Structure with the following elements:
%
%  oversampFact
%        state0
%                   r
%                   e
%                   v
%              omegaB
%       distMat
%
% P ---------- Structure with the following elements:
%
%    quadParams
%     constants
%  sensorParams
%
% OUTPUTS
%
% Q ---------- Structure with the following elements:
%
%          tVec
%         state
%                rMat
%                eMat
%                vMat
%           omegaBMat
%
%+------------------------------------------------------------------------------+
% References:
%
%
% Author:  
%+==============================================================================+

N = length(R.tVec);
dtIn = R.tVec(2) - R.tVec(1);
dtOut = dtIn/S.oversampFact;
RBIk = euler2dcm(S.state0.e);
omegaVec0 = zeros(4,1);
Xk = [S.state0.r;S.state0.v;RBIk(:);S.state0.omegaB;omegaVec0];
statek.RBI = zeros(3,3);
XMat = []; tVec = [];

for kk=1:N-1
  % Populate structs needed by controllers
  Rtc.rIstark = R.rIstar(kk,:)';
  Rtc.vIstark = R.vIstar(kk,:)';
  Rtc.aIstark = R.aIstar(kk,:)';
  Rac.xIstark = R.xIstar(kk,:)';
  statek.rI = Xk(1:3);
  statek.RBI(:) = Xk(7:15);
  statek.vI = Xk(4:6);
  statek.omegaB = Xk(16:18);
  Stc.statek = statek;
  Sac.statek = statek;
  distVeck = S.distMat(kk,:)';
  % Call trajectory and attitude controllers
  [Fk,Rac.zIstark] = trajectoryController(Rtc,Stc,P);
  NBk = attitudeController(Rac,Sac,P);
  % Convert commanded Fk and NBk to commanded voltages
  eaVeck = voltageConverter(Fk,NBk,P);
  tspan = [R.tVec(kk):dtOut:R.tVec(kk+1)]';
  [tVeck,XMatk] = ...
      ode45(@(t,X) quadOdeFunctionHF(t,X,eaVeck,distVeck,P),tspan,Xk);
  if(length(tspan) == 2)
    % Deal with S.oversampFact = 1 case 
    tVec = [tVec; tVeck(1)];
    XMat = [XMat; XMatk(1,:)];
  else
    tVec = [tVec; tVeck(1:end-1)];
    XMat = [XMat; XMatk(1:end-1,:)];
  end
  Xk = XMatk(end,:)';
  % Ensure that RBI remains orthogonal
  if(mod(kk,100) == 0)
   RBIk(:) = Xk(7:15);
   [UR,~,VR]=svd(RBIk);
   RBIk = UR*VR'; Xk(7:15) = RBIk(:);
  end
end
XMat = [XMat;XMatk(end,:)];
tVec = [tVec;tVeck(end,:)];

M = length(tVec);
Q.tVec = tVec;
Q.state.rMat = XMat(:,1:3);
Q.state.vMat = XMat(:,4:6);
Q.state.omegaBMat = XMat(:,16:18);
Q.state.eMat = zeros(M,3);
RBI = zeros(3,3);
for mm=1:M
  RBI(:) = XMat(mm,7:15);
  Q.state.eMat(mm,:) = dcm2euler(RBI)';  
end

end