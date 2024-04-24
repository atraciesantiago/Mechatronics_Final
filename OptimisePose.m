% Define initial gripper orientation
gripper_orientation = [0, 0, 0]; % Identity matrix represents no rotation

% Define object orientation
object_orientation = get_model_pose(model_name); 

% Compute rotation matrix to align gripper's y-axis with object's z-axis
rotation_axis = cross([0, 1, 0], object_orientation); % Compute rotation axis as cross product
rotation_angle = acos(dot([0, 1, 0], object_orientation)); % Compute rotation angle
rotation_matrix = vrrotvec2mat([rotation_axis, rotation_angle]); % Convert axis-angle representation to rotation matrix

% Apply rotation matrix to gripper orientation
aligned_gripper_orientation = rotation_matrix * gripper_orientation;