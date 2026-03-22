function [U_out, L_vec, PriRuntime, OptcostRuntime_iter]=MPC_U_L(L_vec0 , U0, X, VehicleDuOver, params)
N_step=params.N_step; n_car=params.n_car;
options = optimset('MaxIter', 20) ;
u_min=-20; u_max=5;
LBound=u_min*ones(n_car*N_step,1); % [U1; U2; U3]
UBound=u_max*ones(n_car*N_step,1);
A = [] ; B = [] ; Aeq = [] ; Beq = [] ;
[set_L, n_set_L]=set_L123(params); % params.set_L=set_L;
params.n_set_L=n_set_L; 
params.L_vec0=L_vec0;
x_safe_0=5; params.x_safe_0=x_safe_0;
%
startTime1 =tic;
[IndexforOvtk, J_vec]=OvertakingPriority(L_vec0 , X, VehicleDuOver, params);
PriRuntime=toc(startTime1);
%
Pos=X(1:n_car); % Vel=X(n_car+1: 2*n_car);
xA0=Pos(IndexforOvtk(1)); %vA0=Vel(IndexforOvtk(1));
xB0=Pos(IndexforOvtk(2)); %vB0=Vel(IndexforOvtk(2));
xC0=Pos(IndexforOvtk(3)); %vC0=Vel(IndexforOvtk(3));
n_ind=0;
OptcostRuntime_iter=zeros(8,1);
startTime2 =tic;
for lA=1:n_set_L
    for lB=1:n_set_L
        for lC=1:n_set_L
            n_ind=n_ind+1;
            L_vec=L_vec0;
            L_vec(IndexforOvtk(1))=set_L(lA);
            L_vec(IndexforOvtk(2))=set_L(lB);
            L_vec(IndexforOvtk(3))=set_L(lC);
            index(n_ind, :)=[lA, lB, lC]; params.L_vec=L_vec;
            %
            optim_tag=1;
            LA=set_L(lA) ; LB= set_L(lB) ; LC=set_L(lC);
            if abs(xA0-xB0)<x_safe_0 && L_vec0(IndexforOvtk(1))~=L_vec0(IndexforOvtk(2))
                if  LA~=L_vec0(IndexforOvtk(1)) || LB~=L_vec0(IndexforOvtk(2))
                    optim_tag=0;
                end
            end
            if abs(xA0-xC0)<x_safe_0 && L_vec0(IndexforOvtk(1))~=L_vec0(IndexforOvtk(3))
                if  LA~=L_vec0(IndexforOvtk(1)) || LC~=L_vec0(IndexforOvtk(3))
                    optim_tag=0;
                end
            end
            if abs(xC0-xB0)<x_safe_0 && L_vec0(IndexforOvtk(3))~=L_vec0(IndexforOvtk(2))
                if  LC~=L_vec0(IndexforOvtk(3)) || LB~=L_vec0(IndexforOvtk(2))
                    optim_tag=0;
                end
            end
            if optim_tag==1
                startTime3=tic;
                [Uopt(:, n_ind), Fval(n_ind)]=fmincon(@(U)Cost(U,X,params),U0,A,B,Aeq,Beq,LBound,UBound,@(U)NLC(U,X, params),options);
               OptcostRuntime_iter(n_ind)=toc(startTime3);
            else
                Fval(n_ind)=10^15;
            end
        end
    end
end
S_Fval=sum(Fval<10^15);
if S_Fval==0
    error
end
Alllanecost=toc(startTime2);
[J, ind_opt]=min(Fval);
lABC=index(ind_opt, :);
LA=set_L(lABC(1), 1);LB=set_L(lABC(2), 1);LC=set_L(lABC(3), 1);
L_vec=L_vec0;
L_vec(IndexforOvtk(1))=LA;
L_vec(IndexforOvtk(2))=LB;
L_vec(IndexforOvtk(3))=LC;
U_out=Uopt(:, ind_opt);

