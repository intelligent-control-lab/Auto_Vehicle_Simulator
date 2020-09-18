load('traj_platoon.mat');
traj_log = traj_log(23:95,1:2);
load('traj_plan_platoon.mat');
traj_plan = traj_plan(89:4:380,1:4);

track_error = zeros(1,142);

for i = 1:72
    pos = [traj_log(i+1,:) - traj_log(i,:),0];
    plan = [traj_plan(i,3:4) - traj_log(i,:),0];
    error = cross(pos,plan)/norm(plan);
    track_error(i) = abs(error(3));
end

% 
% load('traj_platoon.mat');
% traj_log = traj_log(15:93,1:2);
% load('traj_plan_platoon.mat');
% traj_plan = traj_plan(57:4:372,1:4);
% 
% track_error = zeros(1,79);
% 
% for i = 1:78
%     pos = [traj_log(i+1,:) - traj_log(i,:),0];
%     plan = [traj_plan(i,3:4) - traj_plan(i,1:2),0];
%     error = cross(pos,plan)/norm(plan);
%     track_error(i) = abs(error(3));
% end