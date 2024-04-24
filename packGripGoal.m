function gripGoal = packGripGoal(pos,gripGoal)
     jointWaypointTimes = 3;
     jointWaypoints = [pos]';
     numJoints = size(jointWaypoints,1);
     gripGoal.Trajectory.JointNames = {'robotiq_85_left_knuckle_joint'};
     gripGoal.GoalTolerance.Name = rosmessage('control_msgs/JointTolerance','DataFormat', 'struct');
     gripGoal.GoalTolerance.Name         = gripGoal.Trajectory.JointNames{1};
     gripGoal.GoalTolerance.Position     = 0;
     gripGoal.GoalTolerance.Velocity     = 0.1;
     gripGoal.GoalTolerance.Acceleration = 0.1;

     trajPts = rosmessage('trajectory_msgs/JointTrajectoryPoint');
     trajPts.TimeFromStart = rosduration(jointWaypointTimes);
     trajPts.Positions = jointWaypoints;
     trajPts.Velocities = zeros(size(jointWaypoints));
     trajPts.Accelerations = zeros(size(jointWaypoints));
     trajPts.Effort = 0.1.*ones(size(jointWaypoints));

     gripGoal.Trajectory.Points = trajPts;
end
