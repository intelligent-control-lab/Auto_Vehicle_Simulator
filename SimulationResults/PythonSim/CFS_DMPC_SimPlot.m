clear;
close all;

color = [[0 0.75 1];[1 0 0];[0 1 0];[0.5 0.4 0.8];[0.9 0.9 0.1];[0 0 1]];

%% Intersection
load('CFS_DMPC_Intersection.mat');
traj_log = traj_log(20:70,:);
% Plot traj
[horizon, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = dim_num_veh/2;

figure(1)
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(1),veh_pos(2),'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:horizon-1
        plot(veh_traj(j:j+1,1),veh_traj(j:j+1,2),'Linewidth',1.5,'color',(1-j/2/horizon)*color(i+1,:));
        if mod(j,5)==0
            plot(veh_traj(j,1),veh_traj(j,2),'*','markersize',6,'color',(1-j/2/horizon)*color(i+1,:));
            hold on;
        end
    hold on
    end
end

for i=0:num_veh-1
    veh_pos = traj_log(horizon,2*i+1:2*i+2);
    plot(veh_pos(1),veh_pos(2),'s','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

xlabel('x(m)');
ylabel('y(m)');
legend('Vehicle 1','Vehicle 2','Vehicle 3','Vehicle 4');


%  
load('CFS_DMPC_Intersection_28.mat');
% Plot traj
[num_veh, ~, ~] = size(traj_sol);

figure(2)
for i=1:num_veh
    traj = squeeze(traj_sol(i,:,:));
    plot(traj(1,1),traj(1,2),'^','color',color(i,:),'Linewidth',1.5);
    hold on
end

for i=1:num_veh
        traj = squeeze(traj_sol(i,:,:));
        plot(traj(:,1),traj(:,2),'Linewidth',1.5,'color',color(i,:));
        hold on
end

xlabel('x(m)');
ylabel('y(m)');
legend('Vehicle 1','Vehicle 2','Vehicle 3','Vehicle 4');
axis([-20 20 5 45]);

%  
load('CFS_DMPC_Intersection_30.mat');
% Plot traj
[num_veh, ~, ~] = size(traj_sol);

figure(3)
for i=1:num_veh
    traj = squeeze(traj_sol(i,:,:));
    plot(traj(1,1),traj(1,2),'^','color',color(i,:),'Linewidth',1.5);
    hold on
end

for i=1:num_veh
        traj = squeeze(traj_sol(i,:,:));
        plot(traj(:,1),traj(:,2),'Linewidth',1.5,'color',color(i,:));
        hold on
end

xlabel('x(m)');
ylabel('y(m)');
legend('Vehicle 1','Vehicle 2','Vehicle 3','Vehicle 4');
axis([-20 20 5 45]);



%% Unstructed Road
load('CFS_DMPC_UnstructedRoad.mat');
traj_log = traj_log(20:80,:);
theta = pi/4;
R = [cos(theta) -sin(theta) ; sin(theta) cos(theta)];

% Plot traj
[horizon, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = dim_num_veh/2;

for i=0:num_veh-1
    traj_log(:,2*i+1:2*i+2) = traj_log(:,2*i+1:2*i+2)*R';
end

figure(4)
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(1),veh_pos(2),'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:horizon-1
        plot(veh_traj(j:j+1,1),veh_traj(j:j+1,2),'Linewidth',1.5,'color',(1-j/2/horizon)*color(i+1,:));
        if mod(j,5)==0
            plot(veh_traj(j,1),veh_traj(j,2),'*','markersize',6,'color',(1-j/2/horizon)*color(i+1,:));
            hold on;
        end
    hold on
    end
end

for i=0:num_veh-1
    veh_pos = traj_log(horizon,2*i+1:2*i+2);
    plot(veh_pos(1),veh_pos(2),'s','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

xlabel('x(m)');
ylabel('y(m)');
legend('Vehicle 1','Vehicle 2');


% 
load('CFS_DMPC_UnstructedRoad_25.mat');
theta = pi/4;
R = [cos(theta) -sin(theta) ; sin(theta) cos(theta)];

% Plot traj
[num_veh, horizon, dim] = size(traj_sol);

figure(6)
for i=1:num_veh
    traj = squeeze(traj_sol(i,:,:))*R';
    plot(traj(1,1),traj(1,2),'^','color',color(i,:),'Linewidth',1.5);
    hold on
end

for i=1:num_veh
        traj = squeeze(traj_sol(i,:,:))*R';
        plot(traj(:,1),traj(:,2),'Linewidth',1.5,'color',color(i,:));
        hold on;
end

plot([10.2530534417499,-10.9601545597345],[10.2530446553664,-10.9601550813395],'-.','color',color(2,:));
hold on;
plot([-10.2530534417499,10.9601545597345],[10.2530446553664,-10.9601550813395],'-.','color',color(1,:));

xlabel('x(m)');
ylabel('y(m)');
legend('Vehicle 1','Vehicle 2');
axis([-15 15 -15 0])

% 
load('CFS_DMPC_UnstructedRoad_28.mat');
theta = pi/4;
R = [cos(theta) -sin(theta) ; sin(theta) cos(theta)];

% Plot traj
[num_veh, horizon, dim] = size(traj_sol);

figure(5)
for i=1:num_veh
    traj = squeeze(traj_sol(i,:,:))*R';
    plot(traj(1,1),traj(1,2),'^','color',color(i,:),'Linewidth',1.5);
    hold on
end

for i=1:num_veh
        traj = squeeze(traj_sol(i,:,:))*R';
        plot(traj(:,1),traj(:,2),'Linewidth',1.5,'color',color(i,:));
        hold on
end

plot([10.2530534417499,-10.9601545597345],[10.2530446553664,-10.9601550813395],'-.','color',color(2,:));
hold on;
plot([-10.2530534417499,10.9601545597345],[10.2530446553664,-10.9601550813395],'-.','color',color(1,:));

xlabel('x(m)');
ylabel('y(m)');
legend('Vehicle 1','Vehicle 2');
axis([-15 15 -15 0]);
