%%
% figure;
% figure.WindowState = 'maximized';

% for i = 1:length(Nodes)
%     fprintf('For %d-th node, the final time is %.2f\n', Nodes(i), tf{i});
% end
close all;

global s_p;

save_flag = 0;

%%% prepare for plot
% color_array = cell(4,1);
% color_array{1} = [0.9290 0.6940 0.1250];
% color_array{2} = [0.8500 0.3250 0.0980];
% color_array{3} = [0 0.4470 0.7410];
% color_array{4} = [0.4940 0.1840 0.5560];
% color_array{5} = [0.4660 0.6740 0.1880];
% line_array = cell(4,1);
% line_array{1} = "-";
% line_array{2} = "--";
% line_array{3} = ":";
% line_array{4} = "-.";
% line_array{5} = ":.";
rng(0);
color_array = rand(50,3);

% h = figure();				% 创建图形窗口
% warning('off','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');	% 关闭相关的警告提示（因为调用了非公开接口）
% jFrame = get(h,'JavaFrame');	% 获取底层 Java 结构相关句柄吧
% pause(0.1);					% 在 Win 10，Matlab 2017b 环境下不加停顿会报 Java 底层错误。各人根据需要可以进行实验验证
% set(jFrame,'Maximized',1);	%设置其最大化为真（0 为假）
% pause(0.1);					% 个人实践中发现如果不停顿，窗口可能来不及变化，所获取的窗口大小还是原来的尺寸。各人根据需要可以进行实验验证
% warning('on','MATLAB:HandleGraphics:ObsoletedProperty:JavaFrame');		% 打开相关警告设置

figure;
grid on;
h_2d = gca; % get current axis
axis equal % same len for data units along each axis
if demo_index == 3
    axis([0 300, 50 250]);
elseif demo_index == 4
    axis([-150 150, -150 150]);
    text([x0{1}(1)*s_p+5 x0{2}(1)*s_p+5 x0{3}(1)*s_p+5 x0{4}(1)*s_p+5 x0{5}(1)*s_p-10 x0{6}(1)*s_p-10 x0{7}(1)*s_p-10 x0{8}(1)*s_p-10 ...
          x0{9}(1)*s_p-10 x0{10}(1)*s_p+5 x0{11}(1)*s_p+5 x0{12}(1)*s_p+5 x0{13}(1)*s_p+5 x0{14}(1)*s_p+5 x0{15}(1)*s_p+5 x0{16}(1)*s_p+5], ...
         [x0{1}(2)*s_p+5 x0{2}(2)*s_p+5 x0{3}(2)*s_p+5 x0{4}(2)*s_p+5 x0{5}(2)*s_p+5 x0{6}(2)*s_p+5 x0{7}(2)*s_p+5 x0{8}(2)*s_p+5 ...
          x0{9}(2)*s_p-5 x0{10}(2)*s_p-5 x0{11}(2)*s_p-5 x0{12}(2)*s_p-5 x0{13}(2)*s_p+5 x0{14}(2)*s_p+5 x0{15}(2)*s_p+5 x0{16}(2)*s_p+5], ...
         ["$1$","$2$","$3$","$4$","$5$","$6$","$7$","$8$",...
          "$9$","$10$","$11$","$12$","$13$","$14$","$15$","$16$"], "Interpreter", "latex", "FontSize", 12)
elseif demo_index == 5
    axis([-50 350, 50 700]);
    text([x0{1}(1)*s_p-50 x0{2}(1)*s_p-50 x0{3}(1)*s_p-50 x0{4}(1)*s_p-50 x0{5}(1)*s_p-50 x0{6}(1)*s_p-50 x0{7}(1)*s_p-50 x0{8}(1)*s_p-50 ...
          x0{9}(1)*s_p-50 x0{10}(1)*s_p-50 x0{11}(1)*s_p-50 x0{12}(1)*s_p-50 x0{13}(1)*s_p-50 x0{14}(1)*s_p-50 x0{15}(1)*s_p-50 x0{16}(1)*s_p-50 ...
          x0{17}(1)*s_p-50 x0{18}(1)*s_p-50 x0{19}(1)*s_p-50 x0{20}(1)*s_p-50], ...
         [x0{1}(2)*s_p x0{2}(2)*s_p x0{3}(2)*s_p x0{4}(2)*s_p x0{5}(2)*s_p x0{6}(2)*s_p x0{7}(2)*s_p x0{8}(2)*s_p ...
          x0{9}(2)*s_p x0{10}(2)*s_p x0{11}(2)*s_p x0{12}(2)*s_p x0{13}(2)*s_p x0{14}(2)*s_p x0{15}(2)*s_p x0{16}(2)*s_p ...
          x0{17}(2)*s_p x0{18}(2)*s_p x0{19}(2)*s_p x0{20}(2)*s_p], ...
         ["$1$","$2$","$3$","$4$","$5$","$6$","$7$","$8$","$9$","$10$",...
          "$11$","$12$","$13$","$14$","$15$","$16$","$17$","$18$","$19$","$20$"], "Interpreter", "latex", "FontSize", 12)
elseif demo_index == 6
    axis([200 800, 0 600]);
elseif demo_index == 7
    axis([-100 700, 50 750]);
    text([x0{1}(1)*s_p-50 x0{2}(1)*s_p-50 x0{3}(1)*s_p-50 x0{4}(1)*s_p-50 x0{5}(1)*s_p-50 x0{6}(1)*s_p-50 x0{7}(1)*s_p-50 x0{8}(1)*s_p-50 ...
          x0{9}(1)*s_p-50 x0{10}(1)*s_p-50 x0{11}(1)*s_p-50 x0{12}(1)*s_p-50 x0{13}(1)*s_p-50 x0{14}(1)*s_p-50 x0{15}(1)*s_p-50 x0{16}(1)*s_p-50 ...
          x0{17}(1)*s_p-50 x0{18}(1)*s_p-50 x0{19}(1)*s_p-50 x0{20}(1)*s_p-50], ...
         [x0{1}(2)*s_p x0{2}(2)*s_p x0{3}(2)*s_p x0{4}(2)*s_p x0{5}(2)*s_p x0{6}(2)*s_p x0{7}(2)*s_p x0{8}(2)*s_p ...
          x0{9}(2)*s_p x0{10}(2)*s_p x0{11}(2)*s_p x0{12}(2)*s_p x0{13}(2)*s_p x0{14}(2)*s_p x0{15}(2)*s_p x0{16}(2)*s_p ...
          x0{17}(2)*s_p x0{18}(2)*s_p x0{19}(2)*s_p x0{20}(2)*s_p], ...
         ["$1$","$2$","$3$","$4$","$5$","$6$","$7$","$8$","$9$","$10$",...
          "$11$","$12$","$13$","$14$","$15$","$16$","$17$","$18$","$19$","$20$"], "Interpreter", "latex", "FontSize", 12)
end
lighting gouraud
set(gcf,'Renderer','OpenGL') % Set the property (renderer) of the object (current figure) to value (openGL)
hold(h_2d, 'on')
for i = 1:length(Nodes)
    plot(x0{i}(1)*s_p, x0{i}(2)*s_p, "o", "MarkerFaceColor", color_array(i,:), "MarkerEdgeColor", color_array(i,:));
    plot(x_star{i}(1)*s_p, x_star{i}(2)*s_p, "o", "MarkerFaceColor", color_array(i,:), "MarkerEdgeColor", color_array(i,:));
end
for i = 1:length(constraints)
    plot_circle(constraints(i).Center(1)*s_p, constraints(i).Center(2)*s_p, constraints(i).r*s_p, 0.2, 'r');
end

% can be used as plotting results of each iteration
x_i         = zeros(length(Nodes),length(x{1}));
y_i         = zeros(length(Nodes),length(x{1}));
theta_i     = zeros(length(Nodes),length(x{1}));
x_init_i    = zeros(length(Nodes),length(x_init{1}));
y_init_i    = zeros(length(Nodes),length(x_init{1}));
u_i         = zeros(length(Nodes),length(u{1}));
x_wave_i    = zeros(length(Nodes),length(x{1}));
y_wave_i    = zeros(length(Nodes),length(x{1}));
for i = 1:length(Nodes)
    for k = 1:length(x{i})
        x_i(i,k)        = x{i}{k,1}(1);
        y_i(i,k)        = x{i}{k,1}(2);
        theta_i(i,k)    = x{i}{k,1}(3);
        x_init_i(i,k)        = x_init{i}{k,1}(1);
        y_init_i(i,k)        = x_init{i}{k,1}(2);
        u_i(i,k)        = u{i}{k,1}(1);
        x_wave_i(i,k)   = x_wave{i}{k,1}(1);
        y_wave_i(i,k)   = x_wave{i}{k,1}(2);
    end
end
for i = 1:length(Nodes)
    h_init(i) = plot(x_init_i(i,:)*s_p, y_init_i(i,:)*s_p, '-', 'Color',[color_array(i,:),0.25], "LineWidth", 2.5);
    h(i) = plot(x_i(i,:)*s_p, y_i(i,:)*s_p, '-', 'Color',color_array(i,:), "LineWidth", 2.5);
%     plot(x_wave_i(i,:)*s_p, y_wave_i(i,:)*s_p, '-', 'Color',[color_array(i,:),0.25], "LineWidth", 2.5);
end
if demo_index == 3
    hl = legend([h_init(1), h_init(2), h_init(3), h_init(4), h(1), h(2), h(3), h(4)],'Init-traj of UAV $1$','Init-traj of UAV $2$',...
           'Init-traj of UAV $3$','Init-traj of UAV $4$',...
           'Opti-traj of UAV $1$','Opti-traj of UAV $2$','Opti-traj of UAV $3$','Opti-traj of UAV $4$',...
           'Interpreter', 'latex', 'Fontsize',16,'numcolumns',2);
% elseif demo_index == 5
%     hl = legend([h_init(1), h_init(2), h_init(3), h_init(4), h_init(5), h_init(6), h_init(7), h_init(8), ...
%            h_init(9), h_init(10), h_init(11), h_init(12), h_init(13), h_init(14), h_init(15), h_init(16), ...
%            h(1), h(2), h(3), h(4),h(5), h(6), h(7), h(8),h(9), h(10), h(11), h(12),h(13), h(14), h(15), h(16), ...
%            h(17), h(18), h(19), h(20)],...
%            'Init-traj of UAV $1$','Init-traj of UAV $2$','Init-traj of UAV $3$','Init-traj of UAV $4$',...
%            'Init-traj of UAV $5$','Init-traj of UAV $6$','Init-traj of UAV $7$','Init-traj of UAV $8$',...
%            'Init-traj of UAV $9$','Init-traj of UAV $10$','Init-traj of UAV $11$','Init-traj of UAV $12$',...
%            'Init-traj of UAV $13$','Init-traj of UAV $14$','Init-traj of UAV $15$','Init-traj of UAV $16$',...
%            'Init-traj of UAV $17$','Init-traj of UAV $18$','Init-traj of UAV $19$','Init-traj of UAV $20$',...
%            'Opti-traj of UAV $1$','Opti-traj of UAV $2$','Opti-traj of UAV $3$','Opti-traj of UAV $4$',...
%            'Opti-traj of UAV $5$','Opti-traj of UAV $6$','Opti-traj of UAV $7$','Opti-traj of UAV $8$',...
%            'Opti-traj of UAV $9$','Opti-traj of UAV $10$','Opti-traj of UAV $11$','Opti-traj of UAV $12$',...
%            'Opti-traj of UAV $13$','Opti-traj of UAV $14$','Opti-traj of UAV $15$','Opti-traj of UAV $16$',...
%            'Opti-traj of UAV $17$','Opti-traj of UAV $18$','Opti-traj of UAV $19$','Opti-traj of UAV $20$',...
%            'Interpreter', 'latex', 'Fontsize',16,'numcolumns',2);
elseif demo_index == 6
    hl = legend([h_init(1), h_init(2), h_init(3), h_init(4), h_init(5), ...
           h(1), h(2), h(3), h(4),h(5)],...
           '巡飞弹1初始轨迹','巡飞弹2初始轨迹','巡飞弹3初始轨迹','巡飞弹4初始轨迹',...
           '巡飞弹5初始轨迹',...
           '巡飞弹1优化轨迹','巡飞弹2优化轨迹','巡飞弹3优化轨迹','巡飞弹4优化轨迹',...
           '巡飞弹5优化轨迹',...
           'FontName','Microsft YaHei UI',  'Fontsize',12);
end
if demo_index == 5
    axis([-100 400, 50 750]);
    yticks(50:50:750)
end
xlabel("$x \ (m)$", "Interpreter", "latex", "FontSize", 16);
ylabel("$y \ (m)$", "Interpreter", "latex", "FontSize", 16);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
set(gca,'LooseInset',get(gca,'TightInset'))
% if adp == 0 && save_flag == 1
%     fprintf('\nPlease Press any key to plot the solution.\n');
%     pause;
%     fileout = fullfile(tempdir, "traj_"+num2str(demo_index)+".pdf");
%     % print(gcf,fileout,'-vector','-depsc'); % 设置图片格式、分辨率
%     fig = gcf;
%     fig.PaperPositionMode = 'auto';
%     fig_pos = fig.PaperPosition;
%     fig.PaperSize = [fig_pos(3) fig_pos(4)];
%     print(gcf,fileout,'-vector','-dpdf')
% end
for i = 1:length(Nodes)
    delete(h_init(i))
end
if demo_index == 3
    delete(hl)
    axis([0 300, 70 180]);
elseif demo_index == 4
    axis([-150 150, -150 150]);
elseif demo_index == 6
    delete(hl)
end

% plot the dynamic motion of the nodes 
tf_i        = zeros(length(Nodes),size(tf,2));
for i = 1:length(Nodes)
    for k = 1:size(tf,2)
        tf_i(i,k)       = tf{i,k};
    end
end
cnt=size(x_i,2);
% TS=linspace(0,1,cnt);
% tt=linspace(0,1,3*cnt);
dt = 0.005;
tt = cell(length(Nodes),1);
x_all = cell(length(Nodes),1);
y_all = cell(length(Nodes),1);
theta_all = cell(length(Nodes),1);
TS = cell(length(Nodes),1);
max_ttnum = -inf;
for i = 1:length(Nodes)
    TS{i,1} = linspace(0,tf_i(i,end),cnt);
    tt{i,1} = 0:dt:tf_i(i,end);
    x_all{i} = spline(TS{i,1},x_i(i,:),tt{i,1});
    y_all{i} = spline(TS{i,1},y_i(i,:),tt{i,1});
    theta_all{i} = spline(TS{i,1},theta_i(i,:),tt{i,1});
    if length(tt{i,1}) > max_ttnum
        max_ttnum = length(tt{i,1});
    end
end
for i = 1:length(Nodes)
    length_i = length(tt{i,1});
    if length_i < max_ttnum
        for k = length(tt{i,1}):max_ttnum
            x_all{i}(1,k) = x_all{i}(1,length_i);
            y_all{i}(1,k) = y_all{i}(1,length_i);
            theta_all{i}(1,k) = theta_all{i}(1,length_i);
        end
    end
end
[min_dist, max_dist, index] = cal_dist1(Nodes, Interval, Naibors, x_all, y_all);
fprintf('The min and max dist between naibor before optimizing are %.6f, %.6f. \n',min_dist*s_p, max_dist*s_p);
fprintf('   The collision and connectivity dist between naibor are %.6f, %.6f. \n',col_dist*s_p, con_dist*s_p);

if save_flag == 1
    disp('drawing figures...');
    pause(0.02);
    hObject1 = cell(length(Nodes));
    hObject2 = cell(length(Nodes));
    hold on;
    for k = 1:size(x_all{1},2)
        for i = 1:length(Nodes)
            delete(hObject1{i});
            delete(hObject2{i});
            hObject1{i} = plot_circle(x_all{i}(1,k)*s_p, y_all{i}(1,k)*s_p, col_dist*s_p, 0.2, color_array(i,:));
            hObject2{i} = plot_triangle(x_all{i}(1,k)*s_p, y_all{i}(1,k)*s_p, theta_all{i}(1,k) ,0.10*s_p, 60*pi/180, color_array(i,:));
    %         if k == 1 && i == length(Nodes)
    %             disp('Temple Stop');
    %             pause(0.05);
    %             fileout = fullfile(tempdir, "traj_start_"+num2str(demo_index)+".pdf");
    % %             print(gcf,fileout,'-vector','-depsc'); % 设置图片格式、分辨率
    %             fig = gcf;
    %             fig.PaperPositionMode = 'auto';
    %             fig_pos = fig.PaperPosition;
    %             fig.PaperSize = [fig_pos(3) fig_pos(4)];
    %             print(gcf,fileout,'-vector','-dpdf')
    %         end
            if k == index && i == length(Nodes)
                disp('Temple Stop');
                pause(0.05);
                fileout = fullfile(tempdir, "traj_middle_"+num2str(demo_index)+".pdf");
    %             print(gcf,fileout,'-vector','-depsc'); % 设置图片格式、分辨率
                fig = gcf;
                fig.PaperPositionMode = 'auto';
                fig_pos = fig.PaperPosition;
                fig.PaperSize = [fig_pos(3) fig_pos(4)];
                print(gcf,fileout,'-vector','-dpdf')
            end
        end
        pause(0.001);
    end
    % fileout = fullfile(tempdir, "traj_final_"+num2str(demo_index)+".pdf");
    % % print(gcf,fileout,'-vector','-depsc'); % 设置图片格式、分辨率
    % fig = gcf;
    % fig.PaperPositionMode = 'auto';
    % fig_pos = fig.PaperPosition;
    % fig.PaperSize = [fig_pos(3) fig_pos(4)];
    % print(gcf,fileout,'-vector','-dpdf')
    % pause(0.05);
end


% Plot theta
figure;
hold on;
for i = 1:length(Nodes)
    plot(TS{i,1}, theta_i(i,:)*57.3, '-', 'Color',color_array(i,:), "LineWidth", 2.5);
end
legend('巡飞弹1','巡飞弹2','巡飞弹3','巡飞弹4','巡飞弹5',...
           'FontName','Microsft YaHei UI',  'Fontsize',16)
grid on;
xlabel("Time (s)", "Interpreter", "latex", "FontSize", 16);
ylabel("$\theta \ ({^\circ})$", "Interpreter", "latex", "FontSize", 16);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
set(gca,'LooseInset',get(gca,'TightInset'))

% Plot u
figure;
hold on;
% a = 0:0.5:150; b = 0+zeros(length(a),1); plot(a,b,"-",'Color',[0.5 0.5 0.5]) %蓝色线
% text(4.75,-2,"$0^{\circ}$", "Interpreter", "latex", "FontSize", 12)
% x = 0:0.1:5; y = -500+zeros(length(x),1); plot(x,y,"r-", "LineWidth", 2.5) %蓝色线
for i = 1:length(Nodes)
    h(i) = plot(TS{i,1}, u_i(i,:)*57.3, '-', 'Color',color_array(i,:), "LineWidth", 2.5);
end
% u_max = 30;
% a = 0:0.5:10; b = u_max+zeros(length(a),1); plot(a,b,"k-", "LineWidth", 1.5) %蓝色线
% text(9.0,u_max-0.05,"$\omega_{\max}$", "Interpreter", "latex", "FontSize", 15)
% a = 0:0.5:10; b = -u_max+zeros(length(a),1); plot(a,b,"k-", "LineWidth", 1.5) %蓝色线
% text(8.5,-u_max+0.05,"$-\omega_{\max}$", "Interpreter", "latex", "FontSize", 15)
if demo_index == 3
    legend([h(1), h(2), h(3), h(4)],'UAV $1$','UAV $2$','UAV $3$','UAV $4$',...
           'Interpreter', 'latex', 'Fontsize',16)
elseif demo_index == 6
    legend([h(1), h(2), h(3), h(4), h(5)],'巡飞弹1','巡飞弹2','巡飞弹3','巡飞弹4','巡飞弹5',...
           'FontName','Microsft YaHei UI', 'Fontsize',16)
end
grid on;
xlabel("Time (s)", "Interpreter", "latex", "FontSize", 16);
ylabel("$\omega \ (^{\circ}/s)$", "Interpreter", "latex", "FontSize", 16);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
set(gca,'LooseInset',get(gca,'TightInset'))
% if adp == 0 && save_flag == 1
%     fprintf('\nPlease Press any key to plot the solution.\n');
%     pause;
%     fileout = fullfile(tempdir, "u_"+num2str(demo_index)+".pdf");
%     % print(gcf,fileout,'-vector','-depsc'); % 设置图片格式、分辨率
%     fig = gcf;
%     fig.PaperPositionMode = 'auto';
%     fig_pos = fig.PaperPosition;
%     fig.PaperSize = [fig_pos(3) fig_pos(4)];
%     print(gcf,fileout,'-vector','-dpdf')
% end

% Plot t_f
figure;
hold on;
if demo_index == 6
    for i = 1:length(Nodes)
        a = 0:1:150; b = tf_i(i,end)+zeros(length(a),1); plot(a,b,"k-", "LineWidth", 1.0) %蓝色线
    end
end
for i = 1:length(Nodes)
    h(i) = plot(tf_i(i,:), '-', 'Color',color_array(i,:), "LineWidth", 2.5);
end
if demo_index == 3
    legend([h(1), h(2), h(3), h(4)],'UAV $1$','UAV $2$','UAV $3$','UAV $4$',...
           'Interpreter', 'latex', 'Fontsize',16)
elseif demo_index == 6
    legend([h(1), h(2), h(3), h(4), h(5)],'巡飞弹1','巡飞弹2','巡飞弹3','巡飞弹4','巡飞弹5',...
           'FontName','Microsft YaHei UI', 'Fontsize',16)
end
grid on;
xlabel("Iterations", "Interpreter", "latex", "FontSize", 16);
ylabel("$t_f \ (s)$", "Interpreter", "latex", "FontSize", 16);
ax = gca();
ax.FontSize = 16;
ax.TickLabelInterpreter = "latex";
set(gca,'LooseInset',get(gca,'TightInset'))
% if adp == 0 && save_flag == 1
%     fprintf('\nPlease Press any key to plot the solution.\n');
%     pause;
%     fileout = fullfile(tempdir, "tf_"+num2str(demo_index)+".pdf");
%     % print(gcf,fileout,'-vector','-depsc'); % 设置图片格式、分辨率
%     fig = gcf;
%     fig.PaperPositionMode = 'auto';
%     fig_pos = fig.PaperPosition;
%     fig.PaperSize = [fig_pos(3) fig_pos(4)];
%     print(gcf,fileout,'-vector','-dpdf')
% end



% *********************************************************************** %
% *************************Support Functions***************************** %
% *********************************************************************** %
function [min_dist, max_dist, index] = cal_dist1(Nodes, Interval, Naibors, x_all, y_all)
%%%%% calculate munimum distance
min_dist = Inf;
max_dist = -Inf;
index = 0;
for i = 1:length(Nodes)
    for k = 1:length(x_all{i})
        for j = 2:length(Naibors{i})
            dist_ij = sqrt( (x_all{i}(1,k)-x_all{Naibors{i}(j)}(1,k))^2 + ...
                            (y_all{i}(1,k)-y_all{Naibors{i}(j)}(1,k))^2 );
            if dist_ij < min_dist
                min_dist = dist_ij;
                index = k;
            end
            if dist_ij > max_dist
                max_dist = dist_ij;
            end
        end
    end
end
end

function [min_distance, max_distance] = cal_dist2(Nodes, Interval, Naibors, x_all, y_all)
traj = cell(length(Nodes),1);
for i = 1:length(Nodes)
    traj{i} = [x_all{i}', y_all{i}'];
end
for i = 1:length(Nodes)
    for j = 2:length(Naibors{i})
        % 使用 pdist2 计算点之间的距离
        distances = pdist2(traj{i}, traj{Naibors{i}(j)});
        % 找到最小距离及其索引
        [min_distance, idx_min] = min(distances(:));
        [max_distance, idx_max] = max(distances(:));
        % 将索引转换为坐标索引
        [row, col] = ind2sub(size(distances), idx_min);
        % 最近点的坐标
        nearest_point1 = traj{i}(row, :);
        nearest_point2 = traj{Naibors{i}(j)}(col, :);
        disp('最近点的坐标：');
        disp(nearest_point1);
        disp(nearest_point2);
        disp(['最近距离：', num2str(min_distance)]);
        disp(['最远距离：', num2str(max_distance)]);
    end
end
end

function [h] = plotpyramid(varargin)
% PLOTPYRAMID - Display a 3D-pyramid in the current axes
%
%   PLOTPYRAMID(EDGES,ORIGIN,ALPHA,COLOR) displays a 3D-pyramid in the current axes
%   with the following properties:
%   * ORIGIN: 3-elements vector that defines the start point of the pyramid
%   * DEEP : 1-element vector that defines the length of deep
%   * ANGLE : 3-element vector that defines the length of angle
%   * COLOR : 3-elements vector that defines the faces color of the pyramid
%   * ALPHA : scalar that defines the transparency of the pyramid faces (from 0
%             to 1)
%
% Example:
%   >> plotpyramid([ 2  2  2], 20, 0/180*pi, 0/180*pi, 60/180*pi, 60/180*pi, [1 0 0], .8);
%   >> plotpyramid([10 10 10], 20, 0/180*pi, 0/180*pi, 60/180*pi, 60/180*pi, [0 1 0], .8);
%   >> plotpyramid([20 20 20], 10, 0/180*pi, 0/180*pi, 60/180*pi, 60/180*pi, [0 0 1], .8);

% Default input arguments
inArgs = { ...
  [0 0  0] , ... % Default coordinates of the origin point of the pyramid
  20 , ... % Default edge sizes (x,y and z)
  0/180*pi, ...% Default rotate theta angle of the pyramid
  0/180*pi, ...% Default rotate pusai angle of the pyramid
  45/180*pi, ...% Default open theta angle of the pyramid
  60/180*pi, ...% Default open pusai angle of the pyramid
  [1 0 0]       ... % Default Color for the pyramid
  .2          , ... % Default alpha value for the pyramid's faces
  };

% Replace default input arguments by input values
inArgs(1:nargin) = varargin;

% Create all variables
[origin,edges,rotthe_angle,rotpus_angle,opnthe_angle,opnpus_angle,clr,alpha] = deal(inArgs{:});
    
% calculate coordinates of the four point of the square
square_1x = edges ;
square_2x = edges ;
square_3x = edges ;
square_4x = edges ;

square_1y = -edges*tan(opnpus_angle/2);
square_2y = -edges*tan(opnpus_angle/2);
square_3y = +edges*tan(opnpus_angle/2);
square_4y = +edges*tan(opnpus_angle/2);

square_1z = -edges*tan(opnthe_angle/2);
square_2z = +edges*tan(opnthe_angle/2);
square_3z = +edges*tan(opnthe_angle/2);
square_4z = -edges*tan(opnthe_angle/2);

L = [cos(rotpus_angle)*cos(rotthe_angle), cos(rotthe_angle)*sin(rotpus_angle), -sin(-rotthe_angle);
     -sin(rotpus_angle),            cos(rotpus_angle),           0;
     cos(rotpus_angle)*sin(-rotthe_angle), sin(rotpus_angle)*sin(-rotthe_angle),  cos(rotthe_angle)];
L = L';

square_1 = L*[square_1x; square_1y; square_1z];
square_2 = L*[square_2x; square_2y; square_2z];
square_3 = L*[square_3x; square_3y; square_3z];
square_4 = L*[square_4x; square_4y; square_4z];
% calculate ground coordinates of the point of the square
square_1g = origin + square_1';
square_2g = origin + square_2';
square_3g = origin + square_3';
square_4g = origin + square_4';
    % 网格顶点集合
V = [origin;
     square_1g;
     square_2g;
     square_3g;
     square_4g];
% 网格面片集合
F = { [2 3 4 5], ...
      [1 2 3], ...
      [1 3 4], ...
      [1 4 5], ...
      [1 2 5] };

fa = F{1};
fb = [F{2:end}];
fb = reshape(fb, 3 ,4);
fb = fb';
p = patch('vertices', V, 'faces', fa, 'FaceColor', clr, 'FaceAlpha', alpha);
p = [p,patch('vertices', V, 'faces', fb, 'FaceColor', clr, 'FaceAlpha', alpha)];

% grid on
% view(3);

h = p;
end

function h = plot_triangle(x, y, orientation, depth, angle, color)
% x = 2; y =2;
% orientation = 45*pi/180;

d = depth; %三角形的高度
% angle = angle;  %三角形的角度（等腰三角形的形状）

point1_orign =[0; -d/tan(angle); 1];
point2_orign =[d; 0; 1];
point3_orign =[0; d/tan(angle); 1];

trans =[cos(orientation), -sin(orientation), x;
        sin(orientation), cos(orientation), y;
        0, 0, 1];

point1 =trans*point1_orign;
point2 =trans*point2_orign;
point3 =trans*point3_orign;

triangle_x=[point1(1), point2(1), point3(1), point1(1)];
triangle_y=[point1(2), point2(2), point3(2), point1(2)];
h = fill(triangle_x,triangle_y,color);
end

function h = plot_circle(center_x, center_y, actuator_radius, depth, color)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明
angle_range = 0:pi/40:2*pi;
xx = center_x + actuator_radius*cos(angle_range);
yy = center_y + actuator_radius*sin(angle_range);
cmap = hsv(1); % define colors. You could change `hsv` to `jet`, `cool`, ...
h = patch(xx, yy, cmap, 'facealpha', depth, 'edgecolor', 'none','facecolor', color);
end
