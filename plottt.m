%% Runtime
mm=numel(MPCRuntime); % MPCRuntime(i)
t3=(1:mm)*dT;
for j=1:mm
    MeanOptRuntime(j)=mean(OptcostRuntime_iter(:,j));
%     Sum_OptRuntime(j)=sum(OptcostRuntime_iter(:,j));
end
figure(6) ;
subplot(211), plot(t3 , MPCRuntime * 1.01, 'b', t3, MeanOptRuntime*8, 'r' ,'LineWidth' , 1) ; grid on
ylabel('Run Time(s)','fontsize',12,'fontweight','b') ;
set(gca,'FontSize',12) 
legend('Total Run Time', '(n_s_e_t_,_L)^3 x (Mean MPC Runtime)')
subplot(212), plot(t3, MeanOptRuntime, 'r' ,'LineWidth' , 1) ; grid on
ylabel('Run Time(s)','fontsize',12,'fontweight','b') ;
set(gca,'FontSize',12) 
legend('Mean MPC Runtime')
xlabel('time (s)','fontsize',12,'fontweight','b') ;

figure(7) ;
plot(t3, numOverTak, 'r' ,'LineWidth' , 1) ; grid on
ylabel('N_O_v_e_r _T_a_k_i_n_g','fontsize',12,'fontweight','b') ;
set(gca,'FontSize',12) 
legend('Number of Overtaking')
xlabel('time (s)','fontsize',12,'fontweight','b') ;
%%
t1=(1:n)*dT;
t2=(1:n)*dT;
nn=ones(1, n);
Pos=X(1:n_car,:); Vel=X(n_car+1: 2*n_car, :);

figure(2) ;
subplot(311), plot(t2 , Vel(1,1:n)*3.6, 'b'  ,  t2, params.V_des(1)*3.6*nn,'r', 'LineWidth' , 2) ; grid on
ylabel('V_1 (km/h)','fontsize',12,'fontweight','b') ;
legend('Actual Velocity','Desired Velocity')
set(gca,'FontSize',12) 
% xlim([0 14])
subplot(312), plot(t2 , Vel(2,1:n)*3.6  ,  'b'  ,  t2, params.V_des(2)*3.6*nn,'r',  'LineWidth' , 2) ; grid on
ylabel('V_2 (km/h)','fontsize',12,'fontweight','b') ;
legend('Actual Velocity','Desired Velocity')
set(gca,'FontSize',12) 
subplot(313), plot(t2 , Vel(3,1:n)*3.6  , 'b'  ,  t2, params.V_des(3)*3.6*nn,'r',  'LineWidth' , 2) ; grid on
ylabel('V_3 (km/h)','fontsize',12,'fontweight','b') ;
legend('Actual Velocity','Desired Velocity')
set(gca,'FontSize',12) 
xlabel('time (s)','fontsize',12,'fontweight','b') ;

figure(3) ;
subplot(311), plot(t2 , Vel(4,1:n)*3.6  , 'b'  ,  t2, params.V_des(4)*3.6*nn,'r',  'LineWidth' , 2) ; grid on
ylabel('V_4 (km/h)','fontsize',12,'fontweight','b') ;
legend('Actual Velocity','Desired Velocity')
set(gca,'FontSize',12) 
subplot(312), plot(t2 , Vel(5,1:n)*3.6  , 'b'  ,  t2, params.V_des(5)*3.6*nn,'r',  'LineWidth' , 2) ; grid on
ylabel('V_5 (km/h)','fontsize',12,'fontweight','b') ;
legend('Actual Velocity','Desired Velocity')
set(gca,'FontSize',12) 
subplot(313), plot(t2 , Vel(6,1:n)*3.6  , 'b'  ,  t2, params.V_des(6)*3.6*nn,'r',  'LineWidth' , 2) ; grid on
ylabel('V_6 (km/h)','fontsize',12,'fontweight','b') ;
legend('Actual Velocity','Desired Velocity')
set(gca,'FontSize',12) 
xlabel('time (s)','fontsize',12,'fontweight','b') ;
%%
figure(4) ;
subplot(311), plot(t1 , U(1,:)  , 'LineWidth' , 2) ; grid on
ylabel('U_1 (N)','fontsize',12,'fontweight','b') ;
set(gca,'FontSize',12) 
% xlim([0 14])
subplot(312), plot(t1 , U(2,:)   , 'LineWidth' , 2) ; grid on
ylabel('U_2 (N)','fontsize',12,'fontweight','b') ;
set(gca,'FontSize',12) 
subplot(313), plot(t1 , U(3,:) , 'LineWidth' , 2) ; grid on
ylabel('U_3 (N)','fontsize',12,'fontweight','b') ;
set(gca,'FontSize',12) 
xlabel('time (s)','fontsize',12,'fontweight','b') ;

figure(5) ;
subplot(311), plot(t1 , U(4,:) , 'LineWidth' , 2) ; grid on
ylabel('U_4 (N)','fontsize',12,'fontweight','b') ;
set(gca,'FontSize',12) 
subplot(312), plot(t1 , U(5,:) , 'LineWidth' , 2) ; grid on
ylabel('U_5 (N)','fontsize',12,'fontweight','b') ;
set(gca,'FontSize',12) 
subplot(313), plot(t1 , U(6,:) , 'LineWidth' , 2) ; grid on
ylabel('U_6 (N)','fontsize',12,'fontweight','b') ;
set(gca,'FontSize',12) 
xlabel('time (s)','fontsize',12,'fontweight','b') ;

