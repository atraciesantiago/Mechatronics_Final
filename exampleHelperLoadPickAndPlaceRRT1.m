function [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT1
% exampleHelperLoadPickAndPlaceRRT
% Helper function to load the Franka Emika Panda robot and a simple 
% environment of collision objects placed in the robot model world.
%
% This function is for example purposes and may be removed in a future
% release.

% Copyright 2020-2022 The MathWorks, Inc.

% The Franka Emika Panda's inertial elements are missing. For the purposes
% of motion planning (since dynamics is not involved), it is acceptable to ignore this warning.
warnState = warning("off", "robotics:robotmanip:urdfimporter:IntertialComponentsMissing");
cleanup = onCleanup(@()warning(warnState));
% Load the robot model
%franka = loadrobot("frankaEmikaPanda", "DataFormat","row");
ur5e = loadrobot("universalUR5e",DataFormat="row");

% Open the gripper during planning
% franka.Bodies{10}.Joint.HomePosition = 0.04;
% franka.Bodies{11}.Joint.HomePosition = 0.04;
% franka.Bodies{10}.Joint.PositionLimits = [0.04, 0.04];
% franka.Bodies{11}.Joint.PositionLimits = [0.04, 0.04];
doGrip('place')

% % Calling the checkCollision function on the robot can aid in finding which
% % link-pairs of the robot are in collision.
% % Notice that the robot is in self-collision in its home configuration because link 7 and link 11 collide with
% % link 5, and link 9 is in collision with link 7. 
% [isColliding, sepdist, ~] = checkCollision(franka, homeConfiguration(franka), "Exhaustive", "on","SkippedSelfCollisions","parent");
% [r, c] = find(isnan(sepdist));

% % Modify the home configuration of the robot
% config = homeConfiguration(ur5e);qr = [0 0 pi/2 -pi/2 0 0]
config = homeConfiguration(ur5e);
% config(1) = pi/2;
% config(2) = -pi/2;
% config(3) = 0;
% config(5) = 0;
% config(6) = -pi/2;
% config2 = [1.5708 -1.5708 0 0 0 -1.5708];

% % for i = 1:10
% clearCollision(ur5e.Bodies{3});
% clearCollision(ur5e.Bodies{4});
% clearCollision(ur5e.Bodies{5});
% clearCollision(ur5e.Bodies{6});
% clearCollision(ur5e.Bodies{7});
% % end
% % 
% addCollision(ur5e.Bodies{3}, "cylinder", [0.07, 0.11])
% addCollision(ur5e.Bodies{4}, "cylinder", [0.07, 0.5],transl(-0.2, 0, 0.15)*rpy2tr(0,pi/2,0))
% addCollision(ur5e.Bodies{5}, "cylinder", [0.05, 0.45],transl(-0.15, 0, 0)*rpy2tr(0,pi/2,0))
% addCollision(ur5e.Bodies{6}, "cylinder", [0.045, 0.1],transl(0, 0, -0.02)*rpy2tr(0,0,0))
% addCollision(ur5e.Bodies{7}, "cylinder", [0.045, 0.08],transl(0, 0, 0)*rpy2tr(pi/2,0,0))



% % Replace the link 9's collision cylinder with a close approximation which
% % isn't in collision with link 7
% clearCollision(franka.Bodies{9});
% addCollision(franka.Bodies{9}, "cylinder", [0.07, 0.05], trvec2tform([0.0, 0, 0.025]));

% % Create environment as a set of collision objects.
% bench = collisionBox(0.5, 0.9, 0.05);
% belt1 = collisionBox(1.3, 0.4, 0.235);
% barricade = collisionBox(0.8, 0.03, 0.35);
% 
% TBench = trvec2tform([0.35 0 0.2]);
% TBelt1 = trvec2tform([0 -0.6 0.2]);
% 
% bench.Pose = TBench;
% belt1.Pose = TBelt1;
% barricade.Pose = trvec2tform([0.3, -0.25, 0.4]);
% cylinder1 = collisionCylinder(0.03, 0.1);
% cylinder2 = collisionCylinder(0.03, 0.1);
% cylinder3 = collisionCylinder(0.03, 0.1);
% 
% TCyl = trvec2tform([0.5 0.15 0.278]);
% TCyl2 = trvec2tform([0.52 0 0.278]);
% TCyl3 = trvec2tform([0.4 -0.1 0.2758]);
% 
% cylinder1.Pose = TCyl;
% cylinder2.Pose = TCyl2;
% cylinder3.Pose = TCyl3;
% env = {bench, belt1, cylinder1, cylinder2, cylinder3, barricade};


table = collisionBox (2,2,0.01);
Ttable = trvec2tform([0, 0.8, -0.015]);
table.Pose = Ttable;

box1 = collisionBox (0.15,0.25,0.1);
box2 =collisionBox (0.15,0.25,0.1);
Tbox1 = transl(-0.371,0.0078,(0.1393-0.1))*rpy2tr(0,0,0);
Tbox2 = transl(-0.355,0.32,(0.1643-0.1))*rpy2tr(0,1.57,-0.25);
box1.Pose = Tbox1;
box2.Pose = Tbox2;

gCan3 = collisionCylinder(0.03, 0.1);
TgCan3 = trvec2tform([0.664 0.02 (0.1452-0.1)]);
gCan3.Pose = TgCan3;

rCan2 = collisionCylinder(0.03, 0.1);
TrCan2 = trvec2tform([0.6666 0.0152 (0.2651-0.1)]);
rCan2.Pose = TrCan2;

gCan1 = collisionCylinder(0.03, 0.1);
TgCan1 = trvec2tform([-0.3634 -0.0062 (0.2527-0.1)]);
gCan1.Pose = TgCan1;


env = {table,box1,box2,gCan3,rCan2,gCan1};

end