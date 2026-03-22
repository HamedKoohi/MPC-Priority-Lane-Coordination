function [YR2L , YL2R]  = SigFun(params)
%%
dT=params.dT;
YR=-1.5;
YL=1.5;
k= 2.5;
T = 8/k;              % ~1% to 99% transition
% T = 4/k             % ~10% to 80% transition
% t=linspace(-T/2,T/2, 100);
t=-T/2:dT:T/2;
alpha1=1./(1+exp(-k*t));
YR2L=alpha1 * YL+ (1-alpha1)*YR;
YL2R=alpha1 * YR+ (1-alpha1)*YL;
% plot(t, YR2L, t, YL2R,'r')