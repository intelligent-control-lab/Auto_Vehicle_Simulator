clear;
close all;

color = [[0 0.75 1];[1 0 0];[0 1 0];[0.5 0.4 0.8];[0.9 0.9 0.1];[0 0 1];[0 0.75 1];[1 0 0];[0 1 0];[0.5 0.4 0.8];[0.9 0.9 0.1];[0 0 1]];

%% 
load('CFS_DMPC_Traj.mat');
% Plot traj
[horizon, dim_num_veh] = size(traj_log);
dim = 2;
num_veh = dim_num_veh/2;

figure(1)
% subplot(2,1,1)
for i=0:num_veh-1
    veh_pos = traj_log(1,2*i+1:2*i+2);
    plot(veh_pos(1),veh_pos(2),'o','color',color(i+1,:),'Linewidth',1.5);
    hold on
end

for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:horizon-1
        plot(veh_traj(j:j+1,1),veh_traj(j:j+1,2),'Linewidth',1.5,'color',(1-j/2/horizon)*color(i+1,:));
        if mod(j,15)==0
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
% legend('Vehicle 1','Vehicle 2','Vehicle 3','Vehicle 4');
% axis([-30 30 0 50])



figure(2)
% subplot(2,1,2)
lane_dir = [[0,1] ; [0,-1] ; [1,0] ; [-1,0]];   
T_r = 0.1;
t = 0:T_r:(horizon-2)*T_r;
for i=0:num_veh-1
    vel = [];
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:horizon-1
        vel_long = veh_traj(j+1,:)-veh_traj(j,:);
        vel_dir = vel_long*lane_dir(i+1,:)';
        sig =  sign(vel_dir);
        vel(j) = sig*norm(veh_traj(j+1,:)-veh_traj(j,:))/T_r;
%         plot(veh_traj(j:j+1,1),veh_traj(j:j+1,2),'Linewidth',1.5,'color',(1-j/2/horizon)*color(i+1,:));
%         if mod(j,5)==0
%             plot(veh_traj(j,1),veh_traj(j,2),'*','markersize',6,'color',(1-j/2/horizon)*color(i+1,:));
%             hold on;
    end
    plot(t,vel,'Linewidth',1.5,'color',color(i+1,:));
%     mean(vel)
    hold on
    grid on
end
xlabel('Time (s)');
ylabel('longitudinal velocity (m/s)');
legend('Vehicle 1','Vehicle 2','Vehicle 3','Vehicle 4');
axis([0 6 -40 20])


length = 0;
dim = 2;
[time_step, num] = size(traj_log);
num_veh = num/dim;

for i = 1:num_veh
    for j = 1:time_step-1
        if norm(traj_log(j+1,i*2-1:i*2))>20
            j
            break;
        end
        length = length+norm(traj_log(j+1,i*2-1:i*2)-traj_log(j,i*2-1:i*2));
    end
end

length/num_veh



