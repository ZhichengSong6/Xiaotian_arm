state = load("door_state.txt");
gif_name = "door.gif";
t =  0: 0.002 : (length(state)-1) * 0.002;
% plot(t,state(:,1),'-r','LineWidth',1);
% hold on;
% plot(t,state(:,2),'b','LineWidth',1);
% hold on;
% plot(t,state(:,3),'-r','LineWidth',1);
% hold on;
%% 
joint1 = 1;
joint2 = 2;
joint3 = 3;
joint4 = 4;
joint5 = 5;
joint6 = 6;
joint1_torque = state(:,joint1);
joint2_torque = state(:,joint2);
joint3_torque = state(:,joint3);
joint4_torque = state(:,joint4);
joint5_torque = state(:,joint5);
joint6_torque = state(:,joint6);
%%
joint2_plot = joint2_torque(:);
joint3_plot = joint3_torque(:);
joint5_plot = joint5_torque(:);

% determine whether slow the video
slow_ratio = 1;
% time between each frame(s)
sample_time = 0.04; % 0.04s --> 50 fps
%% plot
%--- plot the first frame ---%
% plot data1
plot(t(1),joint2_plot(1),'-r','LineWidth',2);
hold on;

% % plot data2
plot(t(1),joint3_plot(1),'b','LineWidth',2);
hold on;

plot(t(1),joint5_plot(1),'-k','LineWidth',2);
hold on;

%legend('ee_x', 'ee_y', 'ee_z');

grid on;
xlabel('Time(s)');
ylabel('joint output torque(Nm)');
xlim([0 t(end)]);
ylim([-45 45]);
% set(gca,'XTick',[-0.6:0.4:0.8]);
% set(gca,'YTick',[0:4:t_end]);
set(gca,'FontSize',20,'fontname','times');

%-- set the figure position, length and width --%
set(gcf,'color','white','Position',[400,320,1500,900]); %% position [x, y, length, width]
mark= 1;
%-- record as gif --%
for k = 1:(t(end)/sample_time)
    plot(t(1:(sample_time/0.002)*k),joint2_plot(1:(sample_time/0.002)*k),'-r','LineWidth',2);
    hold on;

    plot(t(1:(sample_time/0.002)*k),joint3_plot(1:(sample_time/0.002)*k),'b','LineWidth',2);
    hold on;

       plot(t(1:(sample_time/0.002)*k),joint5_plot(1:(sample_time/0.002)*k),'-k','LineWidth',2);
    hold on;

    legend('joint2','joint3','joint5');
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