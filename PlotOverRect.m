function out=PlotOverRect(xc, yc, theta)


% Define rectangle parameters
% xc = 5;        % x-coordinate of center
% yc = 4;        % y-coordinate of center
a = 3;         % length (horizontal dimension when theta=0)
b = 1.5;         % width (vertical dimension when theta=0)
% theta = 30;    % rotation angle in degrees

% Calculate corners in local coordinates (centered at origin)
x_local = [-a/2, a/2, a/2, -a/2];
y_local = [-b/2, -b/2, b/2, b/2];

% Create rotation matrix
R = [cosd(theta), -sind(theta); sind(theta), cosd(theta)];

% Rotate and translate corners
corners = R * [x_local; y_local];
x_rotated = corners(1, :) + xc;
y_rotated = corners(2, :) + yc;

% Plot filled rotated rectangle
fill(x_rotated, y_rotated, 'y', 'EdgeColor', 'b', 'LineWidth', 1)

% axis equal
% grid on
% xlim([xc - a, xc + a])
% ylim([yc - b, yc + b])
% title(['Rotated Rectangle: Center (', num2str(xc), ', ', num2str(yc), '), \theta = ', num2str(theta), '°'])

% Mark the center point
% hold on
% plot(xc, yc, 'r*', 'MarkerSize', 10, 'LineWidth', 2)

% Draw orientation line
% line_length = max(a, b)/2;
% x_line = [xc, xc + line_length * cosd(theta)];
% y_line = [yc, yc + line_length * sind(theta)];
% plot(x_line, y_line, 'r-', 'LineWidth', 2)

% legend('Rectangle', 'Center', 'Orientation')