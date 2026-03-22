clc
clear
close all
%
Tf=25;
dT=0.1; params.dT=dT;  params.h=dT;
n=floor(Tf/dT);
params.m=1;  params.d=3; % vehicle length
%%
params.V_des=[80; 80; 75; 75 ; 90; 95]/3.6; %  6 vehicles,
P0=[4 ; 11; 25 ; 15 ; 0 ; -5];
V0=[100 ; 80 ; 80; 70 ; 80; 70]/3.6;
L_vec=[1; 1; 1 ; 0 ; 1 ; 0];
params.WR=3;% Road width
%%
X(:,1)=[P0; V0];
T_pred=5;
n_car=numel(P0);
tspan=T_pred/1;
N_step=floor(T_pred/tspan);
params.T_pred=T_pred; params.N_step=N_step; params.n_car=n_car; params.tspan=tspan;
%
Uvec=zeros(n_car*N_step,1);
% Y(t) for lane changing
for icy=1:n_car
    if L_vec(icy)==1
        Ycars(1,icy)=-1.5;
    else
        Ycars(1,icy)=1.5;
    end
end
%
format bank
figure(1)
for i=1:n
    t=i*dT;
    %
    Pos=X(1:n_car,i); Vel=X(n_car+1: 2*n_car,i);
    L_vec_postproc(:, i)=L_vec;
    % Animation
    Roadplot(Pos);
    set(gca,'FontSize',12) 
    L_vec_des=L_vec;
    Ycars(i+1,:) = YLatdyn (Ycars(i,:) , L_vec_des, params);
    VehicleDuOver=FunDuOver(Ycars(i+1,:) , L_vec_des, params);% during the overtaking
    numOverTak(i)=sum(abs(VehicleDuOver));
    if t>13 && t<15
        VehicleDuOver(6)=0; 
    end
    for ip=1:n_car
        %
        ax=[min(Pos)-10; max(Pos)+10;-4;4];
        %
        [phi, Tet]=FunPhiTeta(L_vec_des(ip), VehicleDuOver(ip), Vel(ip), Ycars(i+1,ip), Ycars(i,ip), dT);
        %
        XA=[Pos(ip); 0; Ycars(i+1,ip); 0; Tet; 0; 0; phi];
        out=car_1(XA,ax, VehicleDuOver(ip), params);  drawnow; grid
        text(Pos(ip), Ycars(i+1,ip), num2str(ip) , 'FontSize', 15)
        hold on
    end
    vel_show=round(Vel'*3.6, 2);
    title (['V=[', num2str(vel_show) ,'] km/h  ' , ',   Time=', num2str(t), 's'], 'FontSize',18)
    ylabel('Y (m)','fontsize',12,'fontweight','b') ;
    xlabel('X (m)','fontsize',12,'fontweight','b') ;
    set(gca,'FontSize',12) 
    hold off
    F(i) = getframe;
    %
    params.Uold=Uvec;
    L_vec0=L_vec;
    starttime0=tic;
    [Uvec, L_vec, PriRuntime(i) , OptcostRuntime_iter(:,i)]=MPC_U_L(L_vec, Uvec, X(:,i), VehicleDuOver, params);
    MPCRuntime(i)=toc(starttime0);
    VehicleDuOver';
    U0=[];
    for icar=1:n_car
        set_U(icar, :)=Uvec((icar-1)*N_step+1 : icar*N_step);
        U0=[U0; set_U(icar, 1)];
    end
    U(:, i)=U0;
    Vel'*3.6;
    U(:, i)';
    X(:,i+1)=SLV(X(:,i),  U(:, i), params);
end
%%
% save('movie','F')
% close all; movie(F,5)
save dataset
plottt

