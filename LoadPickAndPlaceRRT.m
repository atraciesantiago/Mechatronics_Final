function [ur5e,config,env] = LoadPickAndPlaceRRT

% Prepare Workspace
clc
clear

% Load the robot model
ur5e = loadrobot("universalUR5e",DataFormat="row"); 

% Open the gripper during planning
%doGrip('place')

% Set position to home configuration of the robot
config = homeConfiguration(ur5e);

% Check for selfcollisions 
[isColliding, sepdist, ~] = checkCollision(ur5e, homeConfiguration(ur5e), "Exhaustive", "on", "SkippedSelfCollisions","parent");
[r, c] = find(isnan(sepdist)); 

%% Create environment as a set of collision objects.

% Pull positions of objects from gazebo
models = getModels();

% Define dimensions for collision objects
cylinder_diameter = 0.066;
cylinder_height = 0.117;
bottle_diameter = 0.033;
bottle_height = 0.2; % assuming
cube_size = [0.03, 0.03, 0.03]; % [length, width, height]

% Define the dimensions for boxes as [length, width, height]
table_dimensions = [2, 2, 0.01];
box_dimensions = [0.15, 0.25, 0.1]; 
scale_dimensions = [0.15, 0.25, 0.033]; 

% Define dimensions for hollow rectangular prisms % [length, width, visible_height]
wcase_dimensions = [0.4,0.6,0.01]; 
bin_dimensions = [0.2, 0.2, 0.05]; 

% Organize in lookup table for easy access
model_dimensions = struct(...
    'table', table_dimensions, ...
    'unit_box', box_dimensions, ...
    'box', box_dimensions, ...
    'scale', scale_dimensions, ...
    'wcase', wcase_dimensions, ...
    'BlueBin', bin_dimensions, ...
    'GreenBin', bin_dimensions, ...
    'gCan', [cylinder_diameter, cylinder_height], ...
    'rCan', [cylinder_diameter, cylinder_height], ...
    'yCan', [cylinder_diameter, cylinder_height], ...
    'rBottle', [bottle_diameter, bottle_height], ...
    'bBottle', [bottle_diameter, bottle_height], ...
    'yBottle', [bottle_diameter, bottle_height], ...
    'pouch', cube_size ...
    );

% Preallocate a cell array to store collision objects
collision_objects = cell(numel(models.ModelNames), 1);

for i = 1:numel(models.ModelNames)
    model_name = models.ModelNames{i};  
    
    % Get pose of object with respect to the base link
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);
    
    % Extract translation and rotation components
    pose_translation = mat_R_T_M(1:3, 4)';
    pose_rotation = rotm2eul(mat_R_T_M(1:3, 1:3)); % Convert rotation matrix to Euler angles 
    
    % Adjust for the table being on a stand
    z_offset = 0.45; 
    % Ensure offset is only added to table's poses
    if i == 2 || i == 3
        pose_translation = pose_translation + [0, 0, z_offset];
    end

    % Adjust poses to match Matlab axes
    %pose_translation_adj = [pose_translation(2), -pose_translation(1), pose_translation(3)]; % Adjusted translation order

    % Pull pose information into a matrix to be used in planning 
    poses_matrix(i, :) = [pose_translation,  pose_rotation];

% Determine object type based on its name
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
    
% Remove numeric suffix from model name if present
    pure_model_name = regexprep(model_name, '[0-9]', '');

% Create collision object based on object type
switch model_type
    case {'cylinder', 'skinny_cylinder'}
        % For cylindrical objects, dimensions are stored as [diameter, height]
        dimensions = model_dimensions.(pure_model_name);
        if strcmp(model_type, 'cylinder')
            collision_objects{i} = collisionCylinder(dimensions(1)/2, dimensions(2));
        else
            collision_objects{i} = collisionCylinder(dimensions(1)/2, dimensions(2));
        end
    case 'cube'
        % Create collision cube
        collision_objects{i} = collisionBox(cube_size(1), cube_size(2), cube_size(3));
    case 'rectangular_prism'
        % Retrieve dimensions for rectangular prism
        dimensions = model_dimensions.(pure_model_name);
        % Create collision box for rectangular prism
        collision_objects{i} = collisionBox(dimensions(1), dimensions(2), dimensions(3));
    case 'hollow_rectangular_prism'
end

% Set pose of the collision object
    T_object = trvec2tform(pose_translation);
    T_object(1:3, 1:3) = eul2rotm(pose_rotation); % Convert Euler angles to rotation matrix and update the rotation part of the transform
    collision_objects{i}.Pose = T_object;
end

% Initialize a cell array to store collision boxes and cylinders
collision_boxes_and_cylinders = {};

% Iterate through the collision_objects cell array
for i = 1:numel(collision_objects)
    % Check if the element is a collisionBox or a collisionCylinder
    if isa(collision_objects{i}, 'collisionBox') || isa(collision_objects{i}, 'collisionCylinder')
        % If it is, store it in the new cell array
        collision_boxes_and_cylinders{end+1} = collision_objects{i};
    end
end

% Assign the filtered collision objects to env
env = collision_boxes_and_cylinders;

