function [] = traj_plotter(out_name, plot_points, p_total, p_dot_total, p_ddot_total, p_dddot_total)

figure;
subplot(4, 1, 1);
plot3(p_total(1,:),p_total(2,:),p_total(3,:));
title('Position')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;
% draw points
hold on;
for i=1:size(plot_points, 2)
    point_i = plot_points(:, i);
    scatter3(point_i(1, 1), point_i(2, 1), point_i(3, 1));
end

subplot(4, 1, 2);
plot3(p_dot_total(1,:),p_dot_total(2,:),p_dot_total(3,:));
title('Velocity')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(4, 1, 3);
plot3(p_ddot_total(1,:),p_ddot_total(2,:),p_ddot_total(3,:));
title('Acceleration')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(4, 1, 4);
plot3(p_dddot_total(1,:),p_dddot_total(2,:),p_dddot_total(3,:));
title('Jerk')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

arrayfun(@(x) grid(x,'on'), findobj(gcf,'Type','axes')); % grid on su tutti i subplot

img_dimensions = get(0, 'Screensize');
img_width = img_dimensions(3) / 4;
img_height = img_dimensions(4);
set(gcf, 'Position', [img_dimensions(1) img_dimensions(2) img_width img_height]);
export_fig(strcat('E:\uni\magistrale\rvc\lab\images\', out_name), '-pdf', '-transparent');
close;
end

