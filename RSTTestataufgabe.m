close all; 
clc;

%% a)  
myRobot = loadrobot('kukaiiwa7');
myRobot.DataFormat = 'column';
showdetails(myRobot);

%% b)
gripper = importrobot('THWSGripper.urdf');

%% c)
addSubtree(myRobot, 'iiwa_link_ee_kuka', gripper);

%% d)
myFigure = interactiveRigidBodyTree(myRobot, 'Frames', 'off');

%% e)
arbeitsflaeche = collisionBox(1.8, 1.8, 0.1);
arbeitsflaeche.Pose = trvec2tform([0.8, 0.0, -0.05]); 
[~, patchTisch] = show(arbeitsflaeche);
patchTisch.FaceColor = [0.7 0.7 0.7]; 
patchTisch.EdgeColor = 'none';

%% f)
greifobjekt = collisionBox(0.08, 0.08, 0.25);
greifobjekt.Pose = trvec2tform([0.5, -0.2, 0.125]);
[~, patchObj] = show(greifobjekt);
patchObj.FaceColor = [1 0 0]; 
patchObj.EdgeColor = 'none';

%% g)
% Die Position des TCP im Base Frame erhält man über die
% Transformationsmatrix mit
% "getTransform(myRobot, configuration, 'TCP')"
% oder durch "myFigure.MarkerBodyPose", wenn TCP als Marker gesetzt ist.

%% h)
addConfiguration(myFigure);

%% i)
myFigure.Configuration = [-0.367949402401595;0.394313213312805;-0.020558898223765;-1.424033793241539;0.026676768433027;1.338388964494919;-0.472900858001125;0;0];
addConfiguration(myFigure);

%% j)
myFigure.Configuration = [-0.371902891729027;0.542186277683668;-0.019123748837529;-1.600489747116036;0.029363011414590;1.022722676721741;-0.486223187371581;0;0];
addConfiguration(myFigure);

%% k)
myFigure.Configuration = [-0.367949402401595;0.394313213312805;-0.020558898223765;-1.424033793241539;0.026676768433027;1.338388964494919;-0.472900858001125;0;0];
addConfiguration(myFigure);

%% l)
myPose = myFigure.MarkerBodyPose;

%% m)
myPose(1:3,4) = [0.6; 0.3; 0.3];
ik = inverseKinematics('RigidBodyTree', myRobot);
weights = [1 1 1 1 1 1];
initguess = homeConfiguration(myRobot);
[config1, info1] = ik('TCP', myPose, weights, initguess);

%% n)
myFigure.Configuration = config1;
addConfiguration(myFigure);

%% o)
myPose(3,4) = myPose(3,4) - 0.1;
[config2, info2] = ik('TCP', myPose, weights, config1);
myFigure.Configuration = config2;
addConfiguration(myFigure);

myPose(3,4) = myPose(3,4) + 0.1;
[config3, info3] = ik('TCP', myPose, weights, config2);
myFigure.Configuration = config3;
addConfiguration(myFigure);

%% p)
myFigure.Configuration = homeConfiguration(myRobot);
addConfiguration(myFigure);

%% q)
AnzahlKonfigurationen = size(myFigure.StoredConfigurations, 2);
AnzahlSamples = 100 * (AnzahlKonfigurationen - 1);
[q, qd, qdd, tSamp] = trapveltraj(myFigure.StoredConfigurations, AnzahlSamples);

%% r)
rateController = rateControl(AnzahlSamples / (max(tSamp) - tSamp(2)));
for i = 1:AnzahlSamples
    myFigure.Configuration = q(:,i);
    waitfor(rateController);
end

%% s)
% Es handelt sich um eine "MoveJ-artige" Bewegung,
% da die Gelenkwinkel interpoliert werden, nicht der kartesische Pfad (MoveL).

%% t)
% sepDist(14) ist immer "inf", da der TCP-Link in der URDF-Datei
% kein <collision>-Element hat und deshalb beim aufrufen von
% "checkCollision" nicht berücksichtigt wird.

%% u)
minimaler_Abstand_Greifobjekt = nan(1, AnzahlSamples);
for i = 1:AnzahlSamples
    [~, sepDist, ~] = checkCollision(myRobot, q(:,i), {greifobjekt}, ...
        "IgnoreSelfCollision", "on", "Exhaustive", "on");
    minimaler_Abstand_Greifobjekt(i) = mean(sepDist(12:13)) * 100;
end

figure;
plot(tSamp, minimaler_Abstand_Greifobjekt);
xlabel('t in s'); ylabel('Abstand in cm');
title('Abstand der Greiferfinger zum Greifobjekt');

%% v)
clear all; 
