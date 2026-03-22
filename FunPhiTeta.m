function [phi, Tet]=FunPhiTeta(L_vec_des, VehicleDuOver, Velx, Ycars2, Ycars1, dT)
if VehicleDuOver==0
    phi=0;
    Tet=0;
end
if VehicleDuOver~=0
    DX=Velx*dT; Tanteta=(Ycars2-Ycars1) / DX;
    Tet=atan(Tanteta);
    phimax=30*pi/180;
    if L_vec_des==1
        coefphi= abs(Ycars2-(-1.5));
        phi=max(-phimax , -coefphi*phimax);
    elseif L_vec_des==0
        coefphi= abs(Ycars2-1.5);
        phi=min(phimax , coefphi*phimax);
    end
end