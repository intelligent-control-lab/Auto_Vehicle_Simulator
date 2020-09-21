clear;
close all;
color = [[0 0.75 1];[1 0 0];[0 1 0];[0.5 0.4 0.8];[0.9 0.9 0.1];[0 0 1]];


load('traj.mat');
% Plot traj
[num_ts, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = dim_num_veh/2;

figure(1)
% Initial position
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

% Planned traj
load('traj_plan.mat');
n = 0;
traj_plan = traj_plan(125+4*n:128+4*n,:);
num = 4;
for i  = 1:num
   traj = traj_plan(i,:);
   y = [traj(1:2:40)+2];
   x = [traj(2:2:40)]; 
   plot(x,y,'--','Linewidth',1,'color',color(i,:));
   hold on
end

% Position
for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:num_ts-1
        plot(veh_traj(j:j+1,2),veh_traj(j:j+1,1)+2,'Linewidth',1.5,'color',(1-j/2/num_ts)*color(i+1,:));
    hold on
    end
end

% End position
for i=0:num_veh-1
    veh_pos = traj_log(num_ts,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'s','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

% Plot lane
length = 90;
Lane_boundary = [0, 6, 0, 2, 0, -2, 0, -6; ...
                length, 6, length, 2, length, -2, length, -6];
Lane_num = 3;
for i=0:Lane_num
    lane = Lane_boundary(:,2*i+1:2*i+2);
    plot(lane(:,1),lane(:,2),'k');
    hold on
end

xlabel('x(m)');
ylabel('y(m)');
legend('Vehicle 1','Vehicle 2','Vehicle 3','Vehicle 4');
box on;
axis([-10,length+10,-8,8]);



%% Vehicle 1 
load('traj.mat');
% Plot traj
[num_ts, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = 1;

hold on;
figure(5)
% Initial position
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

% Planned traj
load('traj_plan.mat');
n = 4;
traj_plan = traj_plan(33:20:36+20*n,:);
for i  = 1:n
   traj = traj_plan(i,:);
   if i == 1
          y = [traj(1:2:40)+2];
          x = [traj(2:2:40)];
   else
       y = [traj(1:2:40)+2];
       x = [traj(2:2:40)]; 
   end
   plot(x,y,'--','Linewidth',1,'color',color(i+1,:));
   hold on
end

% Position
for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:num_ts-1
        plot(veh_traj(j:j+1,2),veh_traj(j:j+1,1)+2,'Linewidth',1.5,'color',(1-j/2/num_ts)*color(i+1,:));
    hold on
    end
end

% End position
for i=0:num_veh-1
    veh_pos = traj_log(num_ts,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'s','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

% Plot lane
length = 40;
Lane_boundary = [0, 6, 0, 2, 0, -2, 0, -6; ...
                length, 6, length, 2, length, -2, length, -6];
Lane_num = 3;
for i=0:Lane_num
    lane = Lane_boundary(:,2*i+1:2*i+2);
    plot(lane(:,1),lane(:,2),'k');
    hold on
end

xlabel('x(m)');
ylabel('y(m)');
legend('Vehicle 1');
box on;
axis([-10,length+10,-5,3]);


