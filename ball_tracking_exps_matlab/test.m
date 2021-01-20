clc;
clear;
file_name = {'orange','green','blue','pink'};
results = cell(length(file_name),1);
for file_index = 1:1:length(file_name)
    i = 1;
    file_path = strcat('repeatability\',file_name{file_index},'_points_uv_xyz.dat');
    fid = fopen(file_path);
    % time u v x y z
    data = textscan(fid, '%s %f %f %f %f %f');
    fclose(fid);
    for j = 1:1:length(data{2})
        % ignore the point if z is smaller than 400mm
        if data{6}(j) >= 400
            ctime_pressure = data{1}{j}(9:end);
            results{file_index}(i,1) = 1000 * (str2double(ctime_pressure(1:2))*3600 + str2double(ctime_pressure(3:4))*60 + str2double(ctime_pressure(5:6))) + str2double(ctime_pressure(7:9)); 
            results{file_index}(i,2) = data{2}(j); 
            results{file_index}(i,3) = data{3}(j); 
            results{file_index}(i,4) = data{4}(j); 
            results{file_index}(i,5) = data{5}(j); 
            results{file_index}(i,6) = data{6}(j);
            i = i+1;
        end

    end
end

% transform the coordinate
origin = [-16, 54, 686];  %pink
origin_x = [15, 43, 648];   %orange
origin_y = [-70, 43, 651];   %sky blue(green)
origin_z = [-19, 112, 664];  % blue
origin_x = origin_x - origin;
origin_y = origin_y - origin;
origin_z = origin_z - origin;
co_x = origin_x/norm(origin_x);
co_y = origin_y/norm(origin_y);
co_z = origin_z/norm(origin_z);
pre_z = cross(co_x, co_y);
pre_y = cross(-co_x, pre_z);
R_o_c = [co_x;pre_y;pre_z];
origin_camera = R_o_c*(-origin)';
T = eye(4);
T(1:3,1:3) = R_o_c;
T(1:3,4)  = origin_camera;

results_tool = zeros(length(results{3}(:,4)),3);

for i = 1:1:length(results{3}(:,4))
    input = [results{3}(i,4),results{3}(i,5),results{3}(i,6),1]';
    output = T*input;
    results_tool(i,:) = output(1:3);
end




% colors
color = [0.850980392156863 0.325490196078431 0.0980392156862745;
    0 1 0;
    0 0 1
    1 0 1];
% x plot
figure(1)
figure1 = plot3(results{3}(:,4),results{3}(:,5),results{3}(:,6),'.', 'color',color(3,:)); hold on;
grid on;
% legend(figure1,'Fontname', 'Times New Roman','FontSize',7.0);
% set(gca,'XLim',[95 220]);set(gca,'YLim',[-90 105]);
% set(gca,'XTick',95:10:220);set(gca,'YTick',-90:20:105);
title({'Three dimensional positions under the camera coordinate system'});
xlabel('$x$ / mm','Interpreter','latex');ylabel('$y$ / mm', 'Interpreter','latex');zlabel('$z$ / mm','Interpreter','latex');
hold off;
set(get(gca,'XLabel'),'FontSize',14);
set(get(gca,'YLabel'),'FontSize',14);
set(get(gca,'TITLE'),'FontSize',16);
set(gca,'fontsize',10);
% print Pressure&Force_anta_20190723.eps -depsc2 -r1200    
print Balltracking.pdf -dpdf -r1200

figure(2)
figure2 = plot3(results_tool(:,1),results_tool(:,2),results_tool(:,3)+800,'.', 'color',color(3,:)); hold on;
grid on;
% legend(figure1,'Fontname', 'Times New Roman','FontSize',7.0);
% set(gca,'XLim',[95 220]);set(gca,'YLim',[-90 105]);
% set(gca,'XTick',95:10:220);set(gca,'YTick',-90:20:105);
title({'Three dimensional positions under the base coordinate system'});
xlabel('$x$ / mm','Interpreter','latex');ylabel('$y$ / mm', 'Interpreter','latex');zlabel('$z$ / mm','Interpreter','latex');
hold off;
set(get(gca,'XLabel'),'FontSize',14);
set(get(gca,'YLabel'),'FontSize',14);
set(get(gca,'TITLE'),'FontSize',16);
set(gca,'fontsize',10);
% print Pressure&Force_anta_20190723.eps -depsc2 -r1200    
print Balltracking_tool.pdf -dpdf -r1200






