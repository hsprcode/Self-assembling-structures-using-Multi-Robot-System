%% environment
numRobots = 100;
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.5;
env.showTrajectory = false;

%% detectors for all robots
detectors = cell(1,numRobots);
for rIdx = 1:numRobots
    detector = RobotDetector(env,rIdx);
    detector.maxDetections = numRobots;
    detector.maxRange = 1; %SENSOR RANGE 0.75 IS OK
    detector.fieldOfView = pi/3;
    detectors{rIdx} = detector;
end
env.plotSensorLines = false; 

%% Initialization
sampleTime = 0.1;              % Sample time [s]
tVec = 0:sampleTime:15;        % Time array                

% Initialize poses randomly
poses = [((10*(scale-1))+7)*(rand(2,numRobots))+2*ones(2,numRobots);
         pi*rand(1,numRobots)];
     
%% Simulation loop
frame = 1;
vel = zeros(3,numRobots);
for idx = 0:numel(tVec)
    
    % Update the environment
    env(1:numRobots, poses);
    xlim([1 10*scale]);   % Without this, axis resizing can slow things down
    ylim([1 10*scale]);  
    
    M(frame) = getframe(gcf);
    frame = frame + 1;
    
    % Read the sensor and execute the controller for each robot
    for rIdx = 1:numRobots
       detections = step(detectors{rIdx}); 
       vel(:,rIdx) = swarmTeamController(poses,rIdx,detections,potentialimage,scale);
    end
    
    % Discrete integration of pose
    poses = poses + vel*sampleTime;
    
end

%% Robot Controller Logic
function vel = swarmTeamController(poses,rIdx,detections,potentialimage,scale)
    
    % Unpack the robot's pose
    pose = poses(:,rIdx);
    
    w = 2; %    INCREASE STALL ROTATION 1 IS OK

    [px,py] = gradient(potentialimage);
    n = ceil(pose(1)); m = ceil(pose(2));
    f_grad = [px(m,n) py(m,n)];
    vx = (1/(4*scale))*f_grad(1);
    vy = (1/(4*scale))*f_grad(2);    
    
    vel = [vx;vy;w];
    if ~isempty(detections)
            
%             Take the average range and angle
            range = min(detections(:,1));
            angle = mean(detections(:,2));
            
            % Move linearly to maintain a range to the nearest robot
            if range < 1
                v = -10; %AVOID COLLISIONS -10 IS OK
            elseif range >= 1
                v = 10;
            end
            
            % Turn to maintain a heading to the nearest robot
            if angle > pi/2
                w = 2;
            elseif angle < -pi/12
                w = -2;
            
            end
             vel = bodyToWorld([v;0;w],pose);
    end
end


    