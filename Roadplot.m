function out=Roadplot(Pos)
w=3;
L=100;
x1=linspace(min(Pos)-10, max(Pos)+10, 100); y1=x1*0-3.01; y2=(2*w-3.01)*ones(1, numel(x1));
plot(x1, y1,  'r', x1, y2 , 'r', 'LineWidth' , 3)
hold on
y3=y1+w;
plot(x1, y3,'-- b', 'LineWidth' , 3), grid
% hold off
out=[];