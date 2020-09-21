clear;
close all;

color = [[0 0.75 1];[1 0 0];[0 1 0];[0.5 0.4 0.8];[0.9 0.9 0.1];[0 0 1]];

%% Intersection
load('CFS_DMPC_Intersection.mat');
% traj_log = traj_log(20:70,:);
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
%         if mod(j,15)==0
%             plot(veh_traj(j,1),veh_traj(j,2),'*','markersize',6,'color',(1-j/2/horizon)*color(i+1,:));
%             hold on;
%         end
    end
end

for i=0:num_veh-1
    veh_traj = traj_log(:,2*i+1:2*i+2);
    for j=1:horizon-1
        if j==30
            plot(veh_traj(j,1),veh_traj(j,2),'x','markersize',8,'color',(1-j/2/horizon)*color(i+1,:));
            hold on;
        end
        if mod(j,45)==0
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
axis([-30 30 -10 60])


% % Speed
% figure(2)
% T_r = 0.1;
% t = 0:T_r:(horizon-2)*T_r;
% for i=0:num_veh-1
%     vel = [];
%     veh_traj = traj_log(:,2*i+1:2*i+2);
%     for j=1:horizon-1
%         vel(j) = norm(veh_traj(j+1,:)-veh_traj(j,:))/T_r;
% %         plot(veh_traj(j:j+1,1),veh_traj(j:j+1,2),'Linewidth',1.5,'color',(1-j/2/horizon)*color(i+1,:));
% %         if mod(j,5)==0
% %             plot(veh_traj(j,1),veh_traj(j,2),'*','markersize',6,'color',(1-j/2/horizon)*color(i+1,:));
% %             hold on;
%     end
%     plot(t,vel,'Linewidth',1.5,'color',color(i+1,:));
%     hold on
%     grid on
% end
% xlabel('Time (s)');
% ylabel('Speed (m/s)');
% legend('Vehicle 1','Vehicle 2','Vehicle 3','Vehicle 4');


%  
figure(3)
load('CFS_DMPC_Intersection_36.mat');
% Plot traj
[num_veh, ~, ~] = size(traj_sol);

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

plot([2,2],[15,35],'--','color',color(1,:));
hold on;
plot([-2,-2],[15,35],'--','color',color(2,:));
hold on;
plot([-15,15],[23,23],'--','color',color(3,:));
hold on;
plot([-15,15],[27,27],'--','color',color(4,:));
hold on;
xlabel('x(m)');
ylabel('y(m)');
legend('Vehicle 1','Vehicle 2','Vehicle 3','Vehicle 4');
axis([-15 15 15 35]);

% 
% 
% %% Unstructured Road
% load('CFS_DMPC_UnstructuredRoad.mat');
% % traj_log = traj_log(20:80,:);
% 
% % Plot traj
% [horizon, dim_num_veh] = size(traj_log);
% dim = 2;
% num_veh = dim_num_veh/2;
% 
% for i=0:num_veh-1
%     traj_log(:,2*i+1:2*i+2) = traj_log(:,2*i+1:2*i+2);
% end
% 
% figure(4)
% for i=0:num_veh-1
%     veh_pos = traj_log(1,2*i+1:2*i+2);
%     plot(veh_pos(1),veh_pos(2),'o','color',color(i+1,:),'Linewidth',1.5);
%     hold on
% end
% 
% for i=0:num_veh-1
%     veh_traj = traj_log(:,2*i+1:2*i+2);
%     for j=1:horizon-1
%         plot(veh_traj(j:j+1,1),veh_traj(j:j+1,2),'Linewidth',1.5,'color',(1-j/2/horizon)*color(i+1,:));
%         if mod(j,5)==0
%             plot(veh_traj(j,1),veh_traj(j,2),'*','markersize',6,'color',(1-j/2/horizon)*color(i+1,:));
%             hold on;
%         end
%     hold on
%     end
% end
% 
% for i=0:num_veh-1
%     veh_pos = traj_log(horizon,2*i+1:2*i+2);
%     plot(veh_pos(1),veh_pos(2),'s','color',color(i+1,:),'Linewidth',1.5);
%     hold on
% end
% 
% xlabel('x(m)');
% ylabel('y(m)');
% legend('Vehicle 1','Vehicle 2','Vehicle 3');
% axis([-25 25 -25 25]);
% 
% 
% 
% 
% % 
% load('CFS_DMPC_UnstructuredRoad_24.mat');
% 
% % Plot traj
% [num_veh, horizon, dim] = size(traj_sol);
% 
% figure(5)
% for i=1:num_veh
%     traj = squeeze(traj_sol(i,:,:));
%     plot(traj(1,1),traj(1,2),'^','color',color(i,:),'Linewidth',1.5);
%     hold on
% end
% 
% for i=1:num_veh
%         traj = squeeze(traj_sol(i,:,:));
%         plot(traj(:,1),traj(:,2),'Linewidth',1.5,'color',color(i,:));
%         hold on
% end
% 
% plot([20,-20],[0,0],'--','color',color(1,:));
% hold on;
% plot([-10,10],[17.32,-17.32],'--','color',color(2,:));
% hold on;
% plot([-10,10],[-17.32,17.32],'--','color',color(3,:));
% hold on;
% 
% 
% xlabel('x(m)');
% ylabel('y(m)');
% legend('Vehicle 1','Vehicle 2','Vehicle 3');
% axis([-1 1 -1 1]*6);
