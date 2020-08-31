clear;
close all;

color = [[0 0.75 1];[1 0 0];[0 1 0];[0.5 0.4 0.8];[0.9 0.9 0.1];[0 0 1]];


%% Overtaking
load('traj_overtaking.mat');
% Plot traj
[num_ts, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = dim_num_veh/2;

figure(1)
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:num_ts-1
%         patch(veh_traj(:,2),veh_traj(:,1)+2,veh_traj(:,2),'EdgeColor','flat','LineWidth',1,'MarkerFaceColor','flat','FaceColor','none');
        plot(veh_traj(j:j+1,2),veh_traj(j:j+1,1)+2,'Linewidth',1.5,'color',(1-j/2/num_ts)*color(i+1,:));
    hold on
    end
end

for i=0:num_veh-1
    veh_pos = traj_log(num_ts,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'s','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

% Plot lane
length = 140;
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
legend('Vehicle 1','Vehicle 2','Vehicle 3','Vehicle 4','Vehicle 5');
axis([-10,length+10,-8,8])


%% Platoon formation
load('traj_platoon.mat');
% Plot traj
[num_ts, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = dim_num_veh/2;

figure(2)
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:num_ts-1
%         patch(veh_traj(:,2),veh_traj(:,1)+2,veh_traj(:,2),'EdgeColor','flat','LineWidth',1,'MarkerFaceColor','flat','FaceColor','none');
        plot(veh_traj(j:j+1,2),veh_traj(j:j+1,1)+2,'Linewidth',1.5,'color',(1-j/2/num_ts)*color(i+1,:));
    hold on
    end
end

for i=0:num_veh-1
    veh_pos = traj_log(num_ts,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'s','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

% Plot lane
length = 50;
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
axis([-10,length+10,-8,8])


%% Merging
load('traj_merging.mat');
% Plot traj
[num_ts, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = dim_num_veh/2;

figure(3)
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:num_ts-1
%         patch(veh_traj(:,2),veh_traj(:,1)+2,veh_traj(:,2),'EdgeColor','flat','LineWidth',1,'MarkerFaceColor','flat','FaceColor','none');
        plot(veh_traj(j:j+1,2),veh_traj(j:j+1,1)+2,'Linewidth',1.5,'color',(1-j/2/num_ts)*color(i+1,:));
    hold on
    end
end

for i=0:num_veh-1
    veh_pos = traj_log(num_ts,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'s','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

% Plot lane
length = 70;
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
axis([-10,length+10,-8,8])

%% Merging
load('traj_merging2.mat');
% Plot traj
[num_ts, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = dim_num_veh/2;

figure(4)
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:num_ts-1
%         patch(veh_traj(:,2),veh_traj(:,1)+2,veh_traj(:,2),'EdgeColor','flat','LineWidth',1,'MarkerFaceColor','flat','FaceColor','none');
        plot(veh_traj(j:j+1,2),veh_traj(j:j+1,1)+2,'Linewidth',1.5,'color',(1-j/2/num_ts)*color(i+1,:));
    hold on
    end
end

for i=0:num_veh-1
    veh_pos = traj_log(num_ts,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'s','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

% Plot lane
length = 70;
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
axis([-10,length+10,-8,8])


%% Crossing
load('traj_crossing.mat');
% Plot traj
[num_ts, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = dim_num_veh/2;

figure(5)
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:num_ts-1
%         patch(veh_traj(:,2),veh_traj(:,1)+2,veh_traj(:,2),'EdgeColor','flat','LineWidth',1,'MarkerFaceColor','flat','FaceColor','none');
        plot(veh_traj(j:j+1,2),veh_traj(j:j+1,1)+2,'Linewidth',1.5,'color',(1-j/2/num_ts)*color(i+1,:));
        if mod(j,25)==0
            plot(veh_traj(j,2),veh_traj(j,1)+2,'*','markersize',6,'color',(1-j/2/num_ts)*color(i+1,:));
            hold on;
        end
    hold on
    end
end

for i=0:num_veh-1
    veh_pos = traj_log(num_ts,2*i+1:2*i+2);
    plot(veh_pos(2),veh_pos(1)+2,'s','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

% Plot lane
length = 45;
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
legend('Vehicle 1','Vehicle 2');
axis([-10,length+10,-8,8])





%% Planning time
load('planning_time_5.mat')
num = 5;
[~,l] = size(planning_time);
max_total = [];
for i = 1:floor(l/num)
    max_total(i) = sum(planning_time((i-1)*num+1:i*num));
end
max(max_total)


%% Cost
% num = 5
cost =  [ -0.3367531634909222  -0.49395316349781593  -0.6871531635226864 ...
            -0.9163531632574929  -1.181553163360178 ];
sum(cost)

% num = 4
cost =  [ -0.3140531633842349  -0.46525316338754497  -0.6524531633749446  -0.8756531632775719 ];
sum(cost)

% num = 3
cost =  [ -0.3367531634909218  -0.49395316337054174  -0.6871531635253726 ];
sum(cost)

% num = 2
cost =  [ -0.32301388480547544 -0.4766138847442061 ];
sum(cost)
