%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

goHome('qr')
resetWorld
disp('Getting Robot and gCan1 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{20};         % gCan1 is cell {20}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
mat_joint_traj(6) = 0.15;
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%% Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.23; % 0.225 for upright cans tends to slip. 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end
end
if ops('debug')
    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end


%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qr')

%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and rCan1 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{24};         % rCan1 is cell{24}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations

mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
% mat_joint_traj(6) = 0.15;
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%%  Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.23; % 0.225 for upright cans tends to slip. 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end



%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qr')

%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and yCan1 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{27};         % yCan1 is cell {27}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations

mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
% mat_joint_traj(6) = 0.1;
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%%  Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.23; % 0.225 for upright cans tends to slip. 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end

%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qz')

%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and yCan3 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{29};         % yCan3 is cell {29}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations

mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
% mat_joint_traj(6) = 0.15;
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%% Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.23; % 0.225 for upright cans tends to slip. 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end

%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qz')
%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and gCan4 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{23};         % gCan4 is cell {23}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations

mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
% mat_joint_traj(6) = 0.15;
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%%  Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.23; % 0.225 for upright cans tends to slip. 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end


%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qr')

%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and rCan3 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{26};         % rCan3 is cell {226}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations

mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
% mat_joint_traj(6) = 0.15;
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%% Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.23; % 0.225 for upright cans tends to slip. 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end


%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qr')
%% Picking Up Pouches

disp('Getting Robot and pouch1 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{12};         % pouch1 is cell {12}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_R_T_M(3,4)=mat_R_T_M(3,4)-0.02; %for pouch
mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
mat_joint_traj(6) = 0.15; %for pouch 4
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%% Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.585; % 0.585 for pouch 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end


%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
     end  
     %% Return to the ready Position for next object
goHome('qr')
%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and pouch2 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{13};         % gCan1 is cell {13}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_R_T_M(3,4)=mat_R_T_M(3,4)-0.02; %for pouch
mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
mat_joint_traj(6) = 0.15; %for pouch 4
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%% Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.585; % 0.585 for pouch 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end


%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
     end  
     %% Return to the ready Position for next object
goHome('qr')
%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and pouch3 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{14};         % pouch3 is cell {14}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_R_T_M(3,4)=mat_R_T_M(3,4)-0.02; %for pouch
mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
% mat_joint_traj(6) = 0.15; 
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%% Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.585; % 0.585 for pouch 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end


%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
     end  
     %% Return to the ready Position for next object
goHome('qr')
%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and pouch7 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{18};         % pouch7 is cell {18}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_R_T_M(3,4)=mat_R_T_M(3,4)-0.02; %for pouch
mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
% mat_joint_traj(6) = 0.15; 
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%% Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.585; % 0.585 for pouch 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [2.19, 0.37, 1.76, -2.13, 0.00, 0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end


%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
     end  
     %% Return to the ready Position for next object
goHome('qr')
%% Bottles Laying Down
% disp('Getting Robot and rBottle1 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{31};         % rBottle1 is cell {31}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_R_T_M(3,4)=mat_R_T_M(3,4)-0.025; %for laying down cans
mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
mat_joint_traj(6) = 1.57;
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%%  Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.23; % 0.225 for upright cans tends to slip. 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [0.95,-0.81,-1.03,1.84,-0.00,-0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end



%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qr')
%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and rBottle2 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{32};         % rBottle is cell {32}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_R_T_M(3,4)=mat_R_T_M(3,4)-0.025; %for laying down cans
mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));
mat_joint_traj(6) = 1.57;
planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%%  Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.23; % 0.225 for upright cans tends to slip. 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [0.95,-0.81,-1.03,1.84,-0.00,-0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end



%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qr')
%% Standing Bottles
% disp('Getting Robot and yBottle3 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{38};         % yBottle3 is cell {38}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_R_T_M(3,4)=mat_R_T_M(3,4)+0.06; %for standing bottles
mat_traj = mat_R_T_M;
% mat_joint_traj(6) = 0.785;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));

planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%%  Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.515; % 0.515 for standing bottles 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [0.95,-0.81,-1.03,1.84,-0.00,-0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end



%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qr')
%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and yBottle2 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{37};         % yBottle2 is cell {37}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_R_T_M(3,4)=mat_R_T_M(3,4)+0.058; %for standing bottles
mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));

planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%%  Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.52; % 0.515 for standing bottles 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [0.95,-0.81,-1.03,1.84,-0.00,-0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end



%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qz')
%% Load Robot, rosshutdown, rosinit
rosshutdown
rosinit('192.168.118.128',11311,"NodeHost",'192.168.118.1')
 [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT;

%% Define ops dictionary and Load Gazebo and obtain poses
  ops = dictionary();                % Type of global dictionary with all options to facilitate passing of options
    ops("debug")               = 0;     % If set to true visualize traj before running  
    ops("toolFlag")            = 0;     % Include rigidly attached robotiq fingers
    ops("traj_steps")          = 1;     % Num of traj steps
    ops("z_offset")            = 0.3;   % Vertical offset for top-down approach
    ops("traj_duration")       = 3;     % Traj duration (secs) 

disp('Getting Robot and bBottle2 Pose...')
    type = 'gazebo';                            % type can be manual, gazebo, cam, ptcloud
    strcmp(type,'gazebo')                       % string compare 
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{34};         % bBottle2 is cell {34}  
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % mat_R_T_M is the translation of robot to the model(gCan1)
 
   %% Visualize Robot
 if ops('debug')
       figure("Name","Pick and Place Using RRT","Units","normalized","OuterPosition",[0, 0, 1, 1],"Visible","on");
    show(ur5e,config,"Visuals","off","Collisions","on");
    xticks(0:1:4);  % Set x-axis tick values from 0 to 3
    yticks(0:1:4);  % Set y-axis tick values from 0 to 3
    zticks(0:1:4);  % Set z-axis tick values from 0 to 3
    hold on
    for i = 1:length(env)
        show(env{i});
    end
 end
%% Create the RRT Path Planner and specify the robot model and environment, Specify Paramters to be tuned, and define start and goal joint configurations
mat_R_T_M(3,4)=mat_R_T_M(3,4)+0.059; %for standing bottles
mat_traj = mat_R_T_M;
[mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(ur5e,mat_traj,ops('toolFlag'));

planner = manipulatorRRT(ur5e, env);
planner.SkippedSelfCollisions='parent';
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.1;

startConfig = get_current_joint_states;
goalConfig = mat_joint_traj;
rng('default');
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner, path);
[interpStateTraj,c] = size(interpStates);

%% Visualize Path

if ops('debug')
    for i = 1:interpStateTraj
        show(ur5e, interpStates(i,:),...
            "PreservePlot", false,...
            "Visuals","off",...
            "Collisions","on");
        title("Plan 1: MaxConnectionDistance = 0.3")
        drawnow;
    end
end
%%  Create action client, message, populate ROS trajectory goal and send
 % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory', ...
                                           'DataFormat', 'struct');
    
 % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    pick_traj_act_client.FeedbackFcn = [];     
    
 % Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

    [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 
    
    traj_result = traj_result.ErrorCode;
 %%  Create action client for gripper
 grip_action_client = rosactionclient('/gripper_controller/follow_joint_trajectory', ...
                                          'control_msgs/FollowJointTrajectory',...
                                          'DataFormat','struct');
    
 % Create a gripper goal action message
    grip_msg = rosmessage(grip_action_client);

 % Testing if setting FeedbackFcn to 0 minimizes the loss connection
    grip_action_client.FeedbackFcn = []; 

 % Set Grip Pos by default to pick / close gripper
 % gripPos = 0.227; 
    gripPos = 0.515; % 0.515 for standing bottles 

 % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

    %% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    



%% Attach Can to the end effector
% Create can as a rigid body
cylinder1 = env{10};
canBody = rigidBody("myCan");
canJoint = rigidBodyJoint("canJoint");

% Get current pose of the robot hand.
startConfig = get_current_joint_states;
endEffectorPose = getTransform(ur5e,startConfig,"wrist_3_link");

% Place can into the end effector gripper.
setFixedTransform(canJoint,endEffectorPose\cylinder1.Pose); 

% Add collision geometry to rigid body.
addCollision(canBody,cylinder1,inv(cylinder1.Pose));
canBody.Joint = canJoint;

% Add rigid body to robot model.
addBody(ur5e,canBody,"wrist_3_link");

% Remove object from environment.
env(10) = [];

%% Specify Goal COnfig and create the RRT Planner

goalConfig = [0.95,-0.81,-1.03,1.84,-0.00,-0.62];
% planner.EnableConnectHeuristic = false;
path = plan(planner,startConfig,goalConfig);

interpStates = interpolate(planner,path);
[interpStateTraj,~] = size(interpStates);
hold off
%% visualize
if ops('debug')
    show(ur5e,config,"Visuals","off","Collisions","on");
    hold on
    for i = 1:length(env)
        show(env{i});
    end

    % Visulize interpolation of steps
    for i = 1:size(interpStates,1)
        show(ur5e,interpStates(i,:),"PreservePlot", false,"Visuals","off","Collisions","on");
        title("Plan 5: Place the Can")
        drawnow;
        if i == (size(interpStates,1))
            view([80,7])
        end
    end
end



%% Send Path to traj_goal
 
% Convert to trajectory_msgs/FollowJointTrajectory
    disp('Converting to JointTrajectory format...');

     [numSteps, ~] = size(path); % interpStates
    
    traj_goal = convert2ROSPointVec(path, ...
                                    rob_joint_names, ...
                                    numSteps, ...
                                    ops('traj_duration'), ...
                                    traj_goal); % Consider passing in ops directly
    
 % Finally send ros trajectory with traj_steps
    disp('Sending traj to action server...')
    if waitForServer(pick_traj_act_client)
        disp('Connected to action server. Sending goal...')
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [traj_result,state,status] = sendGoalAndWait(pick_traj_act_client,traj_goal);
    end 

    traj_result = traj_result.ErrorCode;

%% Set Grip Pos by default to pick / close gripper
    gripPos = 0; % 0.225 for upright cans tends to slip. 


   % Pack gripper information intro ROS message
    grip_goal = packGripGoal_struct(gripPos,grip_msg);

%% Send action goal
    disp('Sending grip goal...');

     if waitForServer(grip_action_client)
        disp('Connected to action server. Sending goal...')
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    else
        % Re-attempt
        disp('First try failed... Trying again...');
        [res,state,status] = sendGoalAndWait(grip_action_client,grip_goal);
    end    
%% Return to the ready Position for next object
goHome('qr')

