function [ur5e,config,env] = LoadPickAndPlaceRRT

%% Prepare Workspace
disp('Loading UR5e robot...')
clc
clear
ur5e = loadrobot("universalUR5e",DataFormat="row"); % Load the robot model
%doGrip('place') % Open the gripper during planning

%% Set configuration of the robot
disp('Setting configuration to current gazebo position...')

%q_ready = get_current_joint_states;
% config = q_ready; % Set position to home configuration of the robo

config = [pi/2, -pi/2, 0, 0, -pi/2, 0];

%% Check for selfcollisions
disp('Checking for self collisions...')
[isColliding, sepdist, ~] = checkCollision(ur5e, homeConfiguration(ur5e), "Exhaustive", "on", "SkippedSelfCollisions","parent");
[r, c] = find(isnan(sepdist)); 

%% Create environment as a set of collision objects.
disp('Defining collision object dimensions...')
models = getModels(); % Pull positions of objects from gazebo

%% Define dimensions for collision objects

can_diameter = 0.066; 
can_height = 0.117;
bottle_diameter = 0.033;
bottle_height = 0.2; 
cube_size = [0.03, 0.03, 0.03]; % [length, width, height]
table_dimensions = [.8, 1.5, 0.03]; % [length, width, height]
box_dimensions = [0.15, 0.25, 0.1]; % [length, width, height]
scale_dimensions = [0.15, 0.25, 0.033];% [length, width, height] 
wcase_dimensions = [0.6, 0.4, 0.01]; % [length, width, visible_height]
bin_dimensions = [0.2, 0.2, 0.05]; % [length, width, visible_height]
unit_box_dimensions= [0,0,0]; %ignore to avoid collisions 

%% Organize in lookup table for easy access
model_dimensions = struct(...
    'table', table_dimensions, ...
    'unit_box', unit_box_dimensions, ...
    'box', box_dimensions, ...
    'scale', scale_dimensions, ...
    'wcase', wcase_dimensions, ...
    'BlueBin', bin_dimensions, ...
    'GreenBin', bin_dimensions, ...
    'gCan', [can_diameter, can_height], ...
    'rCan', [can_diameter, can_height], ...
    'yCan', [can_diameter, can_height], ...
    'rBottle', [bottle_diameter, bottle_height], ...
    'bBottle', [bottle_diameter, bottle_height], ...
    'yBottle', [bottle_diameter, bottle_height], ...
    'pouch', cube_size ...
    );

%% Obtain pose information
disp('Obtaining current object poses from gazebo...')
collision_objects = cell(numel(models.ModelNames), 1); % Preallocate a cell array to store collision objects
for i = 1:numel(models.ModelNames)
    model_name = models.ModelNames{i};  
    disp(['Creating collision object ', num2str(i), '/', num2str(numel(models.ModelNames))]);
 %% Get pose of object with respect to the base link
    %get_robot_gripper_flag=1;
    z_offset= 0.02; %z_offset variable from get_robot_object_pose_wrt_base_link
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name); % Get pose of object with respect to the base link
 
 %% Extract translation and rotation components
    pose_translation = mat_R_T_M(1:3, 4)'; % Extract translation component
    pose_rotation = rotm2eul(mat_R_T_M(1:3, 1:3)); % Convert rotation matrix to Euler angles
    stand_offset = 0.5 - z_offset; 

    if i == 2 || i == 3 % Ensure offset is only added to table's poses
        pose_translation = pose_translation + [0, 0, stand_offset];
    end
  
    poses_matrix(i, :) = [pose_translation,  pose_rotation];  % Pull pose information into a matrix to be used in planning 

%% Determine object type based on its name
    if contains(model_name, {'gCan', 'rCan', 'yCan'})
        model_type = 'cylinder';
    elseif contains(model_name, {'rBottle', 'bBottle', 'yBottle'})
        model_type = 'skinny_cylinder';
    elseif contains(model_name, 'pouch')
        model_type = 'cube';
    elseif contains(model_name, {'table', 'unit_box', 'box', 'scale'})
        model_type = 'rectangular_prism';
    elseif contains(model_name, {'BlueBin', 'GreenBin', 'wcase'})
        model_type = 'hollow_rectangular_prism';
    else
        % Ignores ground plane and robot
        continue;
    end
    
    pure_model_name = regexprep(model_name, '[0-9]', ''); % Remove numeric suffix from model name if present

%% Create collision object based on object type
switch model_type
    case {'cylinder', 'skinny_cylinder'}
        % For cylindrical objects, dimensions are stored as [diameter, height]
        dimensions = model_dimensions.(pure_model_name); % Retrieve dimensions for cylindrical objects
        if strcmp(model_type, 'cylinder')
            collision_objects{i} = collisionCylinder(dimensions(1)/2, dimensions(2));  % Create collision box for cylinder
        else
            collision_objects{i} = collisionCylinder(dimensions(1)/2, dimensions(2));  % Create collision box for skinny_cylinder
        end
    case 'cube'
        collision_objects{i} = collisionBox(cube_size(1), cube_size(2), cube_size(3)); % Create collision box for cube
    case 'rectangular_prism'
        dimensions = model_dimensions.(pure_model_name); % Retrieve dimensions for rectangular prism
        collision_objects{i} = collisionBox(dimensions(1), dimensions(2), dimensions(3));  % Create collision box for rectangular prism
    case 'hollow_rectangular_prism'  % Create collision box for hollow_rectangular_prism
end

%% Set pose of the collision object

    T_object = trvec2tform(pose_translation);
    T_object(1:3, 1:3) = eul2rotm(pose_rotation); % Convert Euler angles to rotation matrix and update the rotation part of the transform
    collision_objects{i}.Pose = T_object;
end

%% Filter collision objects
disp('Filtering out non-collision objects...')
collision_boxes_and_cylinders = {}; % Initialize a cell array to store collision boxes and cylinders
for i = 1:numel(collision_objects) % Iterate through the collision_objects cell array
    % Check if the element is a collisionBox or a collisionCylinder
    if isa(collision_objects{i}, 'collisionBox') || isa(collision_objects{i}, 'collisionCylinder')
        % If so, store in the new cell array
        collision_boxes_and_cylinders{end+1} = collision_objects{i};
    end
end

%% Assign the filtered collision objects to env

env = collision_boxes_and_cylinders;

disp('Environment defined')

