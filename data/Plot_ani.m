filetime = "joint2_state.txt";
gif_name = "chicken_head.gif";
state = load(filetime);
t =  0: 0.002 : (length(state)-1) * 0.002;

%% select data to plot
% theta_record
%data_no1 = 68;
%y1 = state(:,data_no1);

% ee position
ee_x = 111;
ee_y = 112;
ee_z = 113;
y1 = state(:,ee_x);
y2 = state(:,ee_y);
y3 = state(:,ee_z);
% % base_pitch
% data_no2 = 37;
% y2 = state(:,data_no2);

%% decide the plot time range and slow motion
t_start = 50.5;
t_end = 65.5;
t_total = t_end - t_start;
start_index = t_start/0.002 + 1;
end_index = t_end/0.002;
t_plot = t(start_index:end_index) - t_start;
y1_plot = y1(start_index:end_index);
y2_plot = y2(start_index:end_index);
y3_plot = y3(start_index:end_index);

% determine whether slow the video
slow_ratio = 1;
% time between each frame(s)
sample_time = 0.04; % 0.04s --> 50 fps

%% plot
%--- plot the first frame ---%
% plot data1
plot(t_plot(1),y1_plot(1),'-r','LineWidth',2);
hold on;

% % plot data2
plot(t_plot(1),y2_plot(1),'b','LineWidth',2);
hold on;

plot(t_plot(1),y3_plot(1),'-y','LineWidth',2);
hold on;

%legend('ee_x', 'ee_y', 'ee_z');

grid on;
xlabel('Time(s)');
ylabel('EE position(m)');
xlim([0 t_total]);
ylim([-0.35 0.35]);
% set(gca,'XTick',[-0.6:0.4:0.8]);
% set(gca,'YTick',[0:4:t_end]);
set(gca,'FontSize',20,'fontname','times');

%-- set the figure position, length and width --%
set(gcf,'color','white','Position',[400,320,1000,300]); %% position [x, y, length, width]
mark= 1;
%-- record as gif --%
for k = 1:(t_total/sample_time)
    plot(t_plot(1:(sample_time/0.002)*k),y1_plot(1:(sample_time/0.002)*k),'-r','LineWidth',2);
    hold on;

    plot(t_plot(1:(sample_time/0.002)*k),y2_plot(1:(sample_time/0.002)*k),'b','LineWidth',2);
    hold on;

       plot(t_plot(1:(sample_time/0.002)*k),y3_plot(1:(sample_time/0.002)*k),'-k','LineWidth',2);
    hold on;

    legend('ee_x','ee_y','ee_z');
    %-- record the frame and save --%
    F = getframe(gcf);
    im = frame2im(F);
    [I,map] = rgb2ind(im,256);
    if mark == 1
        imwrite(I, map, gif_name, 'GIF', 'Loopcount', inf, 'DelayTime', sample_time);
        mark = mark + 1;
    else
        imwrite(I, map, gif_name, 'WriteMode', 'append', 'DelayTime', sample_time);
    end
end