function out=SLV(X,  U, params)
dt=params.dT;
h=params.h;
n=floor(dt/h);
h=dt/n;

X(:,1)=X;
for i=1:n
    k1=h*Carplant(X,  U, params); 
    k2=h*Carplant(X+k1/2,  U, params);
    k3=h*Carplant(X+k2/2,  U, params);
    k4=h*Carplant(X+k3,  U, params);
    X=X+1/6*(k1+2*k2+2*k3+k4);
end
out=X;
