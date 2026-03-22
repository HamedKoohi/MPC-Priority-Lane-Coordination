function [J1, J2, J3]=J_colis(xA, xB, xC, params)
% x_safe=6*1.5;
LA=params.LA;
LB=params.LB;
LC=params.LC;
%
N=10;
ep1=0.005;
%% xA xB
alpha=abs(LA-LB);
F1=((xA-xB)/x_safe )^2;
J1=(1-alpha)*( 1 / (F1^N+ep1) );
% J1
%%  xA xC
alpha=abs(LA-LC);
F1=((xA-xC)/x_safe )^2;
J2=(1-alpha)*( 1 / (F1^N+ep1) );
% J2
%%  xB xC
alpha=abs(LB-LC);
F1=((xB-xC)/x_safe )^2;
J3=(1-alpha)*( 1 / (F1^N+ep1) );
% J3