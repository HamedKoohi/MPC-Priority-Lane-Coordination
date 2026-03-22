function VehicleDuOver=FunDuOver(Ycars , L_vec_des, params)
n_car=params.n_car;
VehicleDuOver=zeros(n_car,1);
epsil2=0.2;
for ip=1:n_car
    if L_vec_des(ip)==0
        if Ycars(ip)<=0
            VehicleDuOver(ip)=-1;
        elseif (Ycars(ip)>0 && Ycars(ip)<1.5-epsil2)
            VehicleDuOver(ip)=1;
        end
    elseif L_vec_des(ip)==1
        if (Ycars(ip)<0 && Ycars(ip)>-1.5+epsil2)
            VehicleDuOver(ip)=-1;
        elseif Ycars(ip)>=0
            VehicleDuOver(ip)=1;
        end
    end
end