function J= Cost(U , X, params)
n_car=params.n_car;
dT=0.1;
T_pred=params.T_pred;
n=floor(T_pred/dT);
tspan=params.tspan;
N_step=params.N_step;
V_des=params.V_des;
Uold=params.Uold;
%
for icar=1:n_car
    set_U(icar, :)=U((icar-1)*N_step+1 : icar*N_step);
    set_Uold(icar, :)=Uold((icar-1)*N_step+1 : icar*N_step);
end
%
L_vec0=params.L_vec0;
L_vec=params.L_vec;
%
J=0;
Pos0=X(1:n_car);
Vel0=X(n_car+1: 2*n_car);
x_safe_0=params.x_safe_0; w_xsafe=1/4;
sigma=1.5; N=12; ep1=0.001; w1=500; W2=1000;
%
for it=1:n-1
    %
    t=it*dT;
    stepu=floor(t/tspan)+1;
    U0=[];
    for icar=1:n_car
        U0=[U0; set_U(icar, stepu)];
    end

    dX=Carplant(X, U0, params);  X=X+dX*dT;
    Pos=X(1:n_car);
    Vel=X(n_car+1: 2*n_car);
    J=J+0.1*norm(Vel-V_des)^2;
    % Antilock logic
    J_antiLock=0;
    for icar_A=1:n_car
        LA=L_vec(icar_A);
        if LA==0
            for icar_B=[1:icar_A-1, icar_A+1:n_car]
                LB=L_vec(icar_B);
                if LB==0
                    xA0=Pos0(icar_A); xB0=Pos0(icar_B);
                    if xA0>xB0 && abs(xA0-xB0)<x_safe_0*1.5
                        if V_des(icar_A)<V_des(icar_B)
                            J_antiLock=J_antiLock+1*norm(Vel(icar_A)-V_des(icar_A))^2;
                        end
                    end
                end
            end
        end
    end
    J=J+J_antiLock;
    %
    % collision avoidance must be gauranteed
    for icar_A=1:n_car-1
        for icar_B=icar_A+1:n_car
            LA=L_vec(icar_A); LB=L_vec(icar_B);
            vA0=Vel0(icar_A); vB0=Vel0(icar_B);
            xA0=Pos0(icar_A); xB0=Pos0(icar_B);
            xA=Pos(icar_A); xB=Pos(icar_B);
            J1=0;
            if LA==LB
                x_safe=x_safe_0+w_xsafe*(vB0-vA0)*sign(xA0-xB0); % +w * Vego
                if abs(xB0-xA0) <= x_safe*sigma  &&  xB0>xA0
                    if xA<=xB
                        F1=((xA-xB)/ (x_safe * sigma) )^2;
                        J1=1 / (F1^N+ep1);
                    else  % xA>xB
                        J1=w1*(xA-xB)^2+W2;
                    end
                end
                if abs(xB0-xA0) <= x_safe*sigma  &&  xB0<xA0
                    if xB<=xA
                        F1=((xA-xB)/ (x_safe * sigma) )^2;
                        J1=1 / (F1^N+ep1);
                    else
                        J1=w1*(xA-xB)^2+W2;
                    end
                end
                if abs(xA0-xB0) < x_safe_0
                    J1=J1*1000;
                end
                J=J+J1;
            end
            %
        end
    end
end

J=J+20*norm(L_vec-L_vec0)^2; % lane changing penalty
J=J+200*norm((set_Uold(:, 1)-set_U(:, 1))/params.m)^2;
