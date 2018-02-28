clc
clear all
x = 1:5:20:30
path = [2.0000 0.8415;
    4.0000 4;
    7.0000 1.0000;
    6.0000 0.2879];
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotCurrentLocation initialOrientation];
robotRadius = 0.4;
robot = ExampleHelperRobotSimulator('emptyMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
robot.setRobotPose(robotCurrentPose);
plot(path(:,1), path(:,2),'k--d')
xlim([0 20])
ylim([-10 13])
controller = robotics.PurePursuit
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5;
goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);
controlRate = robotics.Rate(10);
while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = step(controller, robot.getRobotPose());
    
    % Simulate the robot using the controller outputs.
    drive(robot, v, omega);
    robotCurrentPose = robot.getRobotPose();
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    
    waitfor(controlRate);
    
end

