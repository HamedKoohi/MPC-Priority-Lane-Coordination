function Ycars = YLatdyn (Ycars0, L_vec_des, params)
%%
dT=params.dT;
n_car=params.n_car;
k0=+3.5;
x=1/3; b=0.5;
Ycars0=Ycars0*x+b; % Map to [0 1]
d=1;
epsil=0.05;
for i=1:n_car
    if L_vec_des(i)==1
        k=-k0;
    else
        k=k0;
    end
    %
    if Ycars0(i)>=1-epsil
        Ycars0(i)=1-epsil;
    elseif Ycars0(i)<=epsil
        Ycars0(i)=epsil;
    end
    %
    dydt=k*Ycars0(i)*(1-Ycars0(i)/d);
    Ycars(i)=Ycars0(i)+dydt*dT;
end
Ycars=(Ycars-b)/x;  % Map from  [0 1] to [-1.5 1.5]
