%% Only Run Once
% load autoware trajectory message type:
% ros2genmsg(pwd)
% you can verify the build was successful by running:
% ros2 msg show autoware_planning_msgs/Trajectory

%% Environment setup
setenv('ROS_DOMAIN_ID', '0');
setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');

% Only if using Docker:
% setenv('FASTDDS_BUILTIN_TRANSPORTS','UDPv4');

%% Set up Control Node
node = ros2node("/control");

tractor_pose_sub   = ros2subscriber(node, '/status/tractor/pose',   'geometry_msgs/PoseStamped');
tractor_twist_sub  = ros2subscriber(node, '/status/tractor/twist',  'geometry_msgs/TwistStamped');
trailer_pose_sub   = ros2subscriber(node, '/status/trailer/pose',   'geometry_msgs/PoseStamped');
trailer_twist_sub  = ros2subscriber(node, '/status/trailer/twist',  'geometry_msgs/TwistStamped');

trajectory_sub = ros2subscriber( ...
    node, ...
    '/planning/scenario_planning/trajectory', ...
    'autoware_planning_msgs/Trajectory', ...
    History="keeplast");

path_sub = ros2subscriber( ...
    node, ...
    '/planning/scenario_planning/path', ...
    'nav_msgs/Path');

% this is what you will publish control signals (velocity and steering) to
control_pub = ros2publisher( ...
    node, ...
    'control/cmd/control_cmd', ...
    'autoware_control_msgs/Control');

%% helper function to extract yaw (heading) from quaternion
function yaw = quat2yaw(q)
    % geometry_msgs/Quaternion has fields x, y, z, w
    % yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2));
    yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2));
end

%% Receive and plot the global path once
pathMsg = receive(path_sub,  10);    % wait up to 10s for the path
nPts    = numel(pathMsg.poses);
xs      = zeros(nPts,1);
ys      = zeros(nPts,1);
for i=1:nPts
    xs(i) = pathMsg.poses(i).pose.position.x;
    ys(i) = pathMsg.poses(i).pose.position.y;
end

figure; hold on; grid on; axis equal;
hPath    = plot(xs, ys,  'k--','LineWidth',2);
hTraj    = plot(NaN, NaN,'ro-','LineWidth',2,'DisplayName','Trailer Trajectory');
hTractor = plot(NaN, NaN,'bs','MarkerSize',10,'MarkerFaceColor','b','DisplayName','Tractor');
hTrailer = plot(NaN, NaN,'gs','MarkerSize',10,'MarkerFaceColor','g','DisplayName','Trailer');
oriLen       = 5.0;  % length of the heading indicator
hTractorOri  = plot([NaN NaN],[NaN NaN],'b-','LineWidth',2);
hTrailerOri  = plot([NaN NaN],[NaN NaN],'g-','LineWidth',2);
legend([hPath hTraj hTractor hTrailer], {'Global Path','Trajectory','Tractor','Trailer'});
xlabel('x [m]'); ylabel('y [m]');

%% Live update loop
while ishghandle(hPath)
    %─── 1) update the planned trajectory ───────────────────────────────
    try
        trajMsg = receive(trajectory_sub, 0.1);
        nPts    = numel(trajMsg.points);
        trajX   = zeros(1,nPts);
        trajY   = zeros(1,nPts);
        for i = 1:nPts
            trajX(i) = trajMsg.points(i).pose.position.x;
            trajY(i) = trajMsg.points(i).pose.position.y;
        end
        set(hTraj, 'XData', trajX, 'YData', trajY);
    catch
        % no new trajectory this cycle

    end

    %─── 2) update tractor pose and speed ──────────────────────────────
    try
        tp = receive(tractor_pose_sub, 0.1);
        x  = tp.pose.position.x;
        y  = tp.pose.position.y;
        set(hTractor, 'XData', x, 'YData', y);
    catch
    end
    try
        % update tractor heading line
        yawT = quat2yaw(tp.pose.orientation);
        x  = tp.pose.position.x;
        y  = tp.pose.position.y;
        set(hTractorOri, ...
            'XData',[x, x+oriLen*cos(yawT)], ...
            'YData',[y, y+oriLen*sin(yawT)]);
        dummy = 0;
    catch
    end
    try
        tt = receive(tractor_twist_sub, 0.1);
        vT = tt.twist.linear.x;  % forward speed
    catch
        vT = NaN;
    end

    %─── 3) update trailer pose and speed ──────────────────────────────
    try
        lp = receive(trailer_pose_sub, 0.1);
        xt = lp.pose.position.x;
        yt = lp.pose.position.y;
        set(hTrailer, 'XData', xt, 'YData', yt);
    catch
    end
    try
        % update tractor heading line
        yawL = quat2yaw(lp.pose.orientation);
        xt = lp.pose.position.x;
        yt = lp.pose.position.y;
        % update trailer heading line
        set(hTrailerOri, ...
            'XData',[xt, xt+oriLen*cos(yawL)], ...
            'YData',[yt, yt+oriLen*sin(yawL)]);
    catch
    end
    try
        lt = receive(trailer_twist_sub, 0.1);
        vL = lt.twist.linear.x;
    catch
        vL = NaN;
    end

    %─── 4) update title with live speeds ──────────────────────────────
    title(sprintf('Tractor speed: %.2f m/s   |   Trailer speed: %.2f m/s', vT, vL));

    drawnow limitrate;
end
clear;