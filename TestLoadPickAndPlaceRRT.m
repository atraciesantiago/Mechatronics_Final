function [ur5e,config,env] = TestLoadPickAndPlaceRRT
% exampleHelperLoadPickAndPlaceRRT

warnState = warning("off", "robotics:robotmanip:urdfimporter:IntertialComponentsMissing");
cleanup = onCleanup(@()warning(warnState));
ur5e = loadrobot("universalUR5e",DataFormat="row");
doGrip('place')
config = homeConfiguration(ur5e);
table = collisionBox (2,2,0.01);
Ttable = trvec2tform([0, 0.8, -0.015]);
table.Pose = Ttable;

%% wooden case
wcase_base = collisionBox (0.4,0.6,0.01);
Twcase_base = transl(0.45,0.4,(0.1475-0.15))*rpy2tr(0,0,0.25);
wcase_base.Pose = Twcase_base
wcase_side1 = collisionBox (0.15,0.6,0.01);
wcase_side2 = collisionBox (0.15,0.6,0.01);
Twcase_side1 = transl(0.25,0.35,(0.1475-0.15))*rpy2tr(0,1.57,0.25);
wcase_side1.Pose = Twcase_side1
Twcase_side2 = transl(0.65,0.45,(0.1475-0.15))*rpy2tr(0,1.57,0.25);
wcase_side2.Pose = Twcase_side2
wcase_side3 = collisionBox (0.4,0.15,0.01);
wcase_side4 = collisionBox (0.4,0.15,0.01);
Twcase_side3 = transl(0.52,0.1,(0.1475-0.15))*rpy2tr(1.57,0,0.25);
wcase_side3.Pose = Twcase_side3
Twcase_side4 = transl(0.38,0.7,(0.1475-0.15))*rpy2tr(1.57,0,0.25);
wcase_side4.Pose = Twcase_side4

% boxes
box1 = collisionBox (0.15,0.25,0.1);
Tbox1 = transl(-0.371,0.0078,(0.1393-0.1))*rpy2tr(0,0,0);
box1.Pose = Tbox1;
box2 =collisionBox (0.15,0.25,0.1);
Tbox2 = transl(-0.355,0.32,(0.1643-0.1))*rpy2tr(0,1.57,-0.25);
box2.Pose = Tbox2;

z_offset = 0.1;
can = collisionCylinder(0.066, 0.117); 
bottle = collisionCylinder(0.033,0.2);
pouch = collisionBox(0.03, 0.03, 0.03);
scale = collisionBox(0.15, 0.25, 0.033);

% cans
% green
gCan1 = can;
TgCan1 = trvec2tform([-0.3634 -0.0062 (0.2527-z_offset)]);
gCan1.Pose = TgCan1;

gCan2  = can;
TgCan2 = trvec2tform([0.0220 , 0.3216 , (0.1178-z_offset)]);
gCan1.Pose = TgCan2;

gCan3 = can;
TgCan3 = trvec2tform([0.664, 0.02, (0.1452-z_offset)]);
gCan3.Pose = TgCan3;

gCan4 = can;
TgCan4 = trvec2tform([0.5295, 0.4962, (0.1604-z_offset)]);
gCan4.Pose = TgCan4;

% red
rCan1 = can;
TrCan1 = trvec2tform([-0.5029, 0.3953, (0.1433-z_offset)]);
rCan1.Pose = TrCan1;

rCan2 = can;
TrCan2 = trvec2tform([0.6666, 0.0152, (0.2651-z_offset)]);
rCan2.Pose = TrCan2;

rCan3 = can;
TrCan3 = trvec2tform([-0.0323, 0.7994, (0.1431-z_offset)]); 
rCan3.Pose = TrCan3;

% yellow
yCan1 = can;
TyCan1 = trvec2tform([-0.1720, 0.6997, (0.1432-z_offset)]);
yCan1.Pose = TyCan1;

yCan2  = can;
TyCan2 = trvec2tform([0.4808  ,  0.6415 ,   (0.1348-z_offset)]);
yCan2.Pose = TyCan2;

yCan3 = can;
TyCan3 = trvec2tform([0.4499 ,0.4602 ,(0.1604-z_offset)]);
yCan3.Pose = TyCan3;

yCan4  = can;
TyCan4 = trvec2tform([0.4809, 0.6417, (0.1348-z_offset)]);
yCan4.Pose = TyCan4;

Pouches
pouch1 = pouch;
Typouch1 = trvec2tform ([ 0.0330 , 0.5340 , (0.0940-z_offset)]);
pouch1.Pose = Typouch1;

pouch2 = pouch;
Typouch2 = trvec2tform([-0.0410 , 0.5330 , (0.0940-z_offset)]);
pouch2.Pose = Typouch2;

pouch3 = pouch;
Typouch3 = trvec2tform([0.0310, 0.6210 ,   (0.0940-z_offset)]);
pouch3.Pose = Typouch3;

pouch4 = pouch;
Typouch4 = trvec2tform([-0.0420 ,   0.6220   , (0.0940-z_offset)]);
pouch4.Pose = Typouch4;

pouch5 = pouch;
Typouch5 = trvec2tform([-0.2040 ,   0.8860 ,   (0.0940-z_offset)]);
pouch5.Pose = Typouch5;

pouch6 = pouch;
Typouch6 = trvec2tform([-0.2840  ,  0.8660 ,  (0.0940-z_offset)]);
pouch6.Pose = Typouch6;

pouch7 = pouch;
Typouch7 = trvec2tform([0.1310  ,  0.6209  , (0.0942-z_offset)]);
pouch7.Pose = Typouch7;

pouch8 = pouch;
Typouch8 = trvec2tform([-0.1310 ,   0.5330  , (0.0941-z_offset)]);
pouch8.Pose = Typouch8;

scale =  scale;
Tyscale = trvec2tform([-0.3672 ,   0.6600,    (0.1150-z_offset)]);
scale.Pose = Tyscale;

rBottle1 = bottle;
TyrBottle1 = trvec2tform([-0.6194,    0.3200,    (0.1187-z_offset)]);
rBottle1.Pose = TyrBottle1;

rBottle2 = bottle;
TyrBottle2 = trvec2tform([ -0.0000,    0.9198 ,   (0.1185-z_offset)]);
rBottle2.Pose = TyrBottle2;

bBottle1 =  bottle;
TybBottle1 = trvec2tform([0.4602 ,  -0.0702   , (0.1836-z_offset)]);
bBottle1.Pose = TybBottle1;

bBottle2 =  bottle;
TybBottle2 = trvec2tform([-0.2300 ,   0.1734  ,  (0.1836-z_offset)]);
bBottle2.Pose = TyrbBottle2;

bBottle3 =  bottle;
TybBottle3 = trvec2tform([0.2275 ,   0.3630 ,   (0.1188-z_offset)]);
bBottle3.Pose = TybBottle3;

yBottle1 =  bottle;
TyyBottle1 = trvec2tform([-0.5262  , -0.0430   , (0.1185-z_offset)]);
yBottle1.Pose = TyyBottle1;

yBottle2 = bottle;
TyyBottle2 = trvec2tform([ 0.1601 ,   0.7800   , (0.1836-z_offset)]);
yBottle2.Pose = TyyBottle2;

yBottle3 =  bottle;
TyyBottle3 = trvec2tform([ -0.1509 ,   0.4064 ,   (0.1836-z_offset)]);
yBottle3.Pose = TyyBottle3;

yBottle4=  bottle;
TyyBottle4 = trvec2tform([0.6250,    0.3326,    (0.1358-z_offset)]);
yBottle4.Pose = TyyBottle4;

env = {table,wcase_base,wcase_side1,wcase_side2,wcase_side3,wcase_side4,box1,box2,scale,...
    gCan1, gCan2, gCan3,gCan4, rCan1, rCan2, rCan3, yCan1, yCan2, yCan3, yCan4, rBottle1,... 
    rBottle2, bBottle1, bBottle2, bBottle3, yBottle1, yBottle2, yBottle3, yBottle4}; 
end

