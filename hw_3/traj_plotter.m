function [] = traj_plotter(out_name, points, time, pos, vel, acc, jerk, snap)

figure;
subplot(nargin-3, 1, 1);
grid on;
plot(time, pos);
hold on;
for i = 1:size(points, 1)
    plot(points(i, 2), points(i, 1), 'green.','MarkerSize',10);
end
xlabel('Time [s]');
ylabel('Position');

subplot(nargin-3, 1, 2);
grid on;
plot(time, vel);
hold on;
% yline(0, '--black');
for i = 1:size(points, 1)
    xline(points(i, 2), ':green','Alpha',0.5);
end
xlabel('Time [s]');
ylabel('Velocity');

subplot(nargin-3, 1, 3);
grid on;
plot(time, acc);
hold on;
% yline(0, '--black');
for i = 1:size(points, 1)
    xline(points(i, 2), ':green','Alpha',0.5);
end
xlabel('Time [s]');
ylabel('Acceleration');

if nargin > 6
    subplot(nargin-2, 1, 4);
    plot(time, jerk);
    xlabel('Time [s]');
    ylabel('Jerk');
end

if nargin > 7
    subplot(nargin-2, 1, 5);
    plot(time, snap);
    xlabel('Time [s]');
    ylabel('Snap');
end

arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes')); % grid on su tutti i subplot

export_fig(strcat('E:\uni\magistrale\rvc\lab\images\', out_name), '-pdf', '-transparent');
close;
end

