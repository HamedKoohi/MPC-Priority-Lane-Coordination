function [out, J_vec]=OvertakingPriority(L_vec0 , X, VehicleDuOver, params)
n_car=params.n_car;
V_des=params.V_des;
x_safe_0=params.x_safe_0;
Pos=X(1:n_car); Vel=X(n_car+1: 2*n_car);

apha2=1;
apha3=1;
N=2;
w_xsafe=1/4; % 40/5=8 % negavite relative velocity

for iveh=1:n_car
    if VehicleDuOver(iveh)==0
        % j1
        j1=(Vel(iveh)-V_des(iveh))^2;
        % j2
        L_veh=L_vec0(iveh);
        Pos_veh=Pos(iveh);
        dist_back=10^3;
        dist_front=10^3;
        j_back=0;
        j_front=0;
        for j=1:n_car
            if L_vec0(j)==L_veh
                dist=abs(Pos(j)-Pos_veh);% j=iveh
                if Pos(j)<Pos_veh && dist<dist_back
                    dist_back=dist;
                    j_back=j;
                end
                if Pos(j)>Pos_veh && dist<dist_front
                    dist_front=dist;
                    j_front=j;
                end
            end
        end
        if j_back~=0
            x_safe_back=x_safe_0+w_xsafe*(Vel(iveh)-Vel(j_back))*sign(Pos(j_back)-Pos_veh); % +w * Vego
        else
            x_safe_back=0;
        end
        if j_front~=0
            x_safe_front=x_safe_0+w_xsafe*(Vel(iveh)-Vel(j_front))*sign(Pos(j_front)-Pos_veh);
        else
            x_safe_front=0;
        end
        j2=(x_safe_back/dist_back)^N+(x_safe_front/dist_front)^N;
        % j3
        dist_back2=10^3;
        dist_front2=10^3;
        j_back2=0;
        j_front2=0;
        for j=1:n_car
            if L_vec0(j)~=L_veh
                dist=abs(Pos(j)-Pos_veh);% j=iveh
                if Pos(j)<Pos_veh && dist<dist_back2
                    dist_back2=dist;
                    j_back2=j;
                end
                if Pos(j)>Pos_veh && dist<dist_front2
                    dist_front2=dist;
                    j_front2=j;
                end
            end
        end
        if j_back2~=0
            x_safe_back2=x_safe_0+w_xsafe*(Vel(iveh)-Vel(j_back2))*sign(Pos(j_back2)-Pos_veh);
        else
            x_safe_back2=1;
        end
        if j_front2~=0
            x_safe_front2=x_safe_0+w_xsafe*(Vel(iveh)-Vel(j_front2))*sign(Pos(j_front2)-Pos_veh);
        else
            x_safe_front2=1;
        end
        j3=(dist_back2/x_safe_back2)^N+(dist_front2/x_safe_front2)^N;
        %
        J_vec(iveh)=j1+apha2*j2+apha3*j3;
        if dist_back2<x_safe_back2 || dist_front2<x_safe_front2
            J_vec(iveh)=-1; % lane changing is not allowed
        end
    else
        J_vec(iveh)=-1; % lane changing is not allowed
    end
end
[Jvec_sort, Ind]=sort(J_vec);
% J_vec
out=Ind(n_car-2:n_car);% three vehicles
