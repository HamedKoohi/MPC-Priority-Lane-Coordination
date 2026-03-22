function out=car_1(X0, ax, isovertak, params)
out=[];
X=X0(1); Y=X0(3);  theta=X0(5); phi=X0(8);
xmin=ax(1);xmax=ax(2);ymin=ax(3);ymax=ax(4);
d=params.d;
X=X-d/2*cos(theta); Y=Y-d/2*sin(theta);
h=d/4;
w=2;
l_w=d/3;
%%
if isovertak~=0
    a = 3;         % length (horizontal dimension when theta=0)
    b = 1.5;         % width (vertical dimension when theta=0)
    x_local = [-a/2, a/2, a/2, -a/2];
    y_local = [-b/2, -b/2, b/2, b/2];
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    corners = R * [x_local; y_local];
    X1=X+a/2*cos(theta);
    Y1=Y+a/2*sin(theta);
    x_rotated = corners(1, :) + X1;
    y_rotated = corners(2, :) + Y1;
    fill(x_rotated, y_rotated, 'y', 'EdgeColor', 'b', 'LineWidth', 1)
    hold on
end
%%
%------------------------------------------------body
a=[X X-h*sin(theta)];
b=[Y Y+h*cos(theta)];
plot(a,b,'Linewidth',w,'Color','b')
axis equal;
hold on

a=[X X+h*sin(theta)];
b=[Y Y-h*cos(theta)];
plot(a,b,'Linewidth',w,'Color','b')
axis equal;
hold on

a=[X-h*sin(theta) X-h*sin(theta)+d*cos(theta)];
b=[Y+h*cos(theta) Y+h*cos(theta)++d*sin(theta)];
plot(a,b,'Linewidth',w,'Color','b')
axis equal;
hold on

a=[X+h*sin(theta) X+h*sin(theta)+d*cos(theta)];
b=[Y-h*cos(theta) Y-h*cos(theta)+d*sin(theta)];
plot(a,b,'Linewidth',w,'Color','b')
axis equal;
hold on

a=[X+h*sin(theta)+d*cos(theta) X-h*sin(theta)+d*cos(theta)];
b=[Y-h*cos(theta)+d*sin(theta) Y+h*cos(theta)++d*sin(theta)];
plot(a,b,'Linewidth',w,'Color','b')
axis equal;
hold on
%--------------------------------------rear wheel
d_w=w;
e_r_r=d/5;
e_r_s=d/7;

e1=e_r_r-l_w/2;
e2=e_r_r+l_w/2;
a=[X+e1*cos(theta)-e_r_s*sin(theta) X+e2*cos(theta)-e_r_s*sin(theta)];
b=[Y+e1*sin(theta)+e_r_s*cos(theta) Y+e2*sin(theta)+e_r_s*cos(theta)];
plot(a,b,'LineWidth',d_w,'Color','r')
axis equal;
hold on

a=[X+e1*cos(theta)+e_r_s*sin(theta) X+e2*cos(theta)+e_r_s*sin(theta)];
b=[Y+e1*sin(theta)-e_r_s*cos(theta) Y+e2*sin(theta)-e_r_s*cos(theta)];
plot(a,b,'LineWidth',d_w,'Color','r')
axis equal;
% axis([xmin xmax ymin ymax])
hold on
%--------------------------front wheel
e_f_s=e_r_s;
e_f_f=e_r_r;
B=phi+theta;

X1=X+(d-e_f_f)*cos(theta)-e_f_s*sin(theta)-l_w/2*cos(B);
X2=X+(d-e_f_f)*cos(theta)-e_f_s*sin(theta)+l_w/2*cos(B);
Y1=Y+(d-e_f_f)*sin(theta)+e_f_s*cos(theta)-l_w/2*sin(B);
Y2=Y+(d-e_f_f)*sin(theta)+e_f_s*cos(theta)+l_w/2*sin(B);
a=[X1 X2];
b=[Y1 Y2];
plot(a,b,'LineWidth',d_w,'Color','r')
axis equal;
% 
X1=X+(d-e_f_f)*cos(theta)+e_f_s*sin(theta)-l_w/2*cos(B);
X2=X+(d-e_f_f)*cos(theta)+e_f_s*sin(theta)+l_w/2*cos(B);
Y1=Y+(d-e_f_f)*sin(theta)-e_f_s*cos(theta)-l_w/2*sin(B);
Y2=Y+(d-e_f_f)*sin(theta)-e_f_s*cos(theta)+l_w/2*sin(B);
a=[X1 X2];
b=[Y1 Y2];
plot(a,b,'LineWidth',d_w,'Color','r')
axis equal;
axis([xmin xmax ymin ymax])
%%
hold off
