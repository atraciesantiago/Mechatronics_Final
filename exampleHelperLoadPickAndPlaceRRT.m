function [ur5e,config,env] = exampleHelperLoadPickAndPlaceRRT
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

% Open the gripper during planning  %in matlab the joint is fixed
% ur5e.Bodies{10}.Joint.HomePosition = 0.04;
% ur5e.Bodies{11}.Joint.HomePosition = 0.04;
% ur5e.Bodies{10}.Joint.PositionLimits = [0.04, 0.04];
% ur5e.Bodies{11}.Joint.PositionLimits = [0.04, 0.04];
% doGrip('place')

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
% % config2 = [1.5708 -1.5708 0 0 0 -1.5708];
% 
% % % for i = 1:10
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

greenbin =collisionBox (0.5,0.5,0.5);
Tgreenbin = transl(-0.5,-0.3654,-0.43)*rpy2tr(0,0,1.57);
greenbin.Pose =Tgreenbin;

bluebin =collisionBox (0.5,0.5,0.5);
Tbluebin = transl(0.5,-0.3654,-0.43)*rpy2tr(0,0,1.57);
bluebin.Pose =Tbluebin;

%wooden case
wcase_base = collisionBox (0.3,0.6,0.01);
Twcase_base = transl(0.5,0.4,(0.1475-0.15))*rpy2tr(0,0,0.25);
wcase_base.Pose = Twcase_base;

wcase_side1 = collisionBox (0.15,0.6,0.01);
wcase_side2 = collisionBox (0.15,0.6,0.01);
Twcase_side1 = transl(0.35,0.35,(0.1475-0.15))*rpy2tr(0,1.57,0.25);
wcase_side1.Pose = Twcase_side1;
Twcase_side2 = transl(0.65,0.45,(0.1475-0.15))*rpy2tr(0,1.57,0.25);
wcase_side2.Pose = Twcase_side2;

wcase_side3 = collisionBox (0.3,0.15,0.01);
wcase_side4 = collisionBox (0.3,0.15,0.01);
Twcase_side3 = transl(0.58,0.1,(0.1475-0.15))*rpy2tr(1.57,0,0.25);
wcase_side3.Pose = Twcase_side3;
Twcase_side4 = transl(0.42,0.7,(0.1475-0.15))*rpy2tr(1.57,0,0.25);
wcase_side4.Pose = Twcase_side4;

% Boxes and Scale
box1 = collisionBox (0.15,0.25,0.1);
box2 =collisionBox (0.15,0.25,0.1);
Tbox1 = transl(-0.371,0.0078,(0.1393-0.1))*rpy2tr(0,0,0);
Tbox2 = transl(-0.355,0.32,(0.1643-0.1))*rpy2tr(0,1.57,-0.25);
box1.Pose = Tbox1;
box2.Pose = Tbox2;

scale = collisionBox(0.15, 0.25, 0.033);
Tscale = transl(-0.3672,0.66,(0.1150-0.1))*rpy2tr(0,0,0.7);
scale.Pose = Tscale;

%Green Cans
gCan1 = collisionCylinder(0.03, 0.1);
TgCan1 = trvec2tform([-0.3634 -0.0062 (0.2527-0.1)]);
gCan1.Pose = TgCan1;

gCan2 = collisionCylinder(0.03, 0.1);
TgCan2 = transl(0.022,0.3216,(0.1178-0.1))*rpy2tr(1.58,0,1.58);
gCan2.Pose = TgCan2;

gCan3 = collisionCylinder(0.03, 0.1);
TgCan3 = trvec2tform([0.664 0.02 (0.1452-0.1)]);
gCan3.Pose = TgCan3;

gCan4 = collisionCylinder(0.03, 0.1);
TgCan4 = trvec2tform([0.5295 0.4962 (0.1604-0.1)]);
gCan4.Pose = TgCan4;

%Red Cans
rCan1 = collisionCylinder(0.03, 0.1);
TrCan1 = trvec2tform([-0.5029 0.3953 (0.1433-0.1)]);
rCan1.Pose = TrCan1;

rCan2 = collisionCylinder(0.03, 0.1);
TrCan2 = trvec2tform([0.6666 0.0152 (0.2651-0.1)]);
rCan2.Pose = TrCan2;

rCan3 = collisionCylinder(0.03, 0.1);
TrCan3 = transl(-0.0322,0.8,(0.1173-0.1))*rpy2tr(1.4865,1.5593,0);
rCan3.Pose = TrCan3;

%Yellow Can
yCan1 = collisionCylinder(0.03, 0.1);
TyCan1 = trvec2tform([-0.1720 0.6997 (0.1432-0.1)]);
yCan1.Pose = TyCan1;

yCan2 = collisionCylinder(0.03, 0.1);
TyCan2 = transl(0.4819,0.6495,(0.1349-0.1))*rpy2tr(1.57,-1,-2.17);
yCan2.Pose = TyCan2;

yCan3 = collisionCylinder(0.03, 0.1);
TyCan3 = trvec2tform([0.4499 0.4602 (0.1604-0.1)]);
yCan3.Pose = TyCan3;

yCan4 = collisionCylinder(0.03, 0.1);
TyCan4 = transl(0.4199,0.3601,(0.1349-0.1))*rpy2tr(1.57,-1,-2.17);
yCan4.Pose = TyCan4;

% Pouches
pouch4 = collisionBox (0.03,0.03,0.03);
Tpouch4 = transl(-0.042,0.6220,(0.0944-0.1))*rpy2tr(0,0,0);
pouch4.Pose = Tpouch4;

pouch1 = collisionBox (0.03,0.03,0.03);
Tpouch1 = transl(0.0330 , 0.5340 , (0.0940-0.1))*rpy2tr(0,0,0);
pouch1.Pose = Tpouch1;

pouch2 = collisionBox (0.03,0.03,0.03);
Tpouch2 = transl(-0.0410 , 0.5330 , (0.0940-0.1))*rpy2tr(0,0,0);
pouch2.Pose = Tpouch2;

pouch3 = collisionBox (0.03,0.03,0.03);
Tpouch3 = transl(0.0310, 0.6210 ,(0.0940-0.1))*rpy2tr(0,0,0);
pouch3.Pose = Tpouch3;

pouch5 = collisionBox (0.03,0.03,0.03);
Tpouch5 = transl(-0.2040 ,0.8860 ,(0.0940-0.1))*rpy2tr(0,0,0);
pouch5.Pose = Tpouch5;

pouch6 = collisionBox (0.03,0.03,0.03);
Tpouch6 = transl(-0.2840,0.8660 ,(0.0940-0.1))*rpy2tr(0,0,0);
pouch6.Pose = Tpouch6;

pouch7 = collisionBox (0.03,0.03,0.03);
Tpouch7 = transl(0.1310  ,  0.6209  , (0.0942-0.1))*rpy2tr(0,0,0);
pouch7.Pose = Tpouch7;

pouch8 = collisionBox (0.03,0.03,0.03);
Tpouch8 = transl(-0.1310 ,   0.5330  , (0.0941-0.1))*rpy2tr(0,0,0);
pouch8.Pose = Tpouch8;

%Yellowe Bottles
yBottle1 = collisionCylinder(0.02, 0.14);
TyBottle1 =transl(-0.5262,-0.0430,(0.1187-0.1))*rpy2tr(1.57,1.46,0);
yBottle1.Pose = TyBottle1;

ybottle2 = collisionCylinder(0.02, 0.14);
Tybottle2 = trvec2tform([0.1601 0.78 (0.1836-0.1)]);
ybottle2.Pose = Tybottle2;

ybottle3 = collisionCylinder(0.02, 0.14);
Tybottle3 = trvec2tform([-0.1509 0.4064 (0.1836-0.1)]);
ybottle3.Pose = Tybottle3;

yBottle4 = collisionCylinder(0.02, 0.14);
TyBottle4 =transl(0.6317,-0.1429,(0.1188-0.1))*rpy2tr(0,1.57,1.58);
yBottle4.Pose = TyBottle4;

%Red Bottle
rBottle1 = collisionCylinder(0.02, 0.14);
TrBottle1 =transl(-0.6195,0.3200,(0.1187-0.1))*rpy2tr(1.57,0,0);
rBottle1.Pose = TrBottle1;

rBottle2 = collisionCylinder(0.02, 0.14);
TrBottle2 =transl(0,0.9198,(0.1185-0.1))*rpy2tr(1.58,0,1.58);
rBottle2.Pose = TrBottle2;

%BlueBottles
bBottle1 = collisionCylinder(0.02, 0.14);
TbBottle1 = trvec2tform([0.4602 -0.0702 (0.1836-0.1)]);
bBottle1.Pose = TbBottle1;

bBottle2 = collisionCylinder(0.02, 0.14);
TbBottle2 = transl(-0.23,0.1733,(0.1836-0.1))*rpy2tr(0,0,0);
bBottle2.Pose = TbBottle2;

bBottle3 = collisionCylinder(0.02, 0.14);
TbBottle3 = transl(0.2274,0.3630,(0.1188-0.1))*rpy2tr(0,1.49,1.58);
bBottle3.Pose = TbBottle3;

env = {table,wcase_base,wcase_side1,wcase_side2,wcase_side3,wcase_side4,box1,box2,gCan3,gCan1,gCan2,rCan1,rCan2,yCan1,rCan3,yCan2,yCan3,yCan4,gCan4,pouch1,pouch2,pouch3,pouch4,pouch5,pouch6,pouch7,pouch8,ybottle3,yBottle1,ybottle2,bBottle1,rBottle1,rBottle2,bBottle3,yBottle4,bBottle2,scale,greenbin,bluebin};

end

