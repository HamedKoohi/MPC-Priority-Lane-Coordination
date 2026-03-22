function dX=Carplant(X, U, params)
n_car=params.n_car;
% Pos=X(1:n_car);
Vel=X(n_car+1: 2*n_car); % X(:,1)=[P0; V0];
%
dX=[Vel ; U/params.m];