function [k_sig, T_LaneC, Lx_LaneC]=Traj_Sigm(params)
d = params.WR;             % lane width [m]
v =params.V;               % vehicle speed [m/s]
ay_max = 2;           % max lateral acceleration [m/s^2]
jy_max = 4;         % max lateral jerk [m/s^3]
%  t0 = 0;               % center of lane change
% logistic jerk constant (derived from analysis)
Cjerk = 0.096;        
%% ================== DESIGN SIGMOID PARAMETER ==================
% ---- acceleration constraint ----
k_acc = sqrt((6*sqrt(3)*ay_max)/d);
% ---- jerk constraint ----
k_jerk = (jy_max/(Cjerk*d))^(1/3);
% ---- choose admissible value ----
k = min(k_acc, k_jerk);
%% ================== MANEUVER TIME ==================
% T = 8/k;              % ~1% to 99% transition
T = 4/k;              % ~10% to 80% transition
L = v*T;              % lane change distance
%%
k_sig=k;
T_LaneC=T;
Lx_LaneC=L;
