%%%This code takes as inputs robot location and target location (any target), and
%%%calculates how much the robot needs to turn and advance. I NOW NEED TO
%%%SET IT TO DO THAT. The second part checks if the target can be reached
%%%by a straight line and outputs a matrix of nodes through which the robot
%%%can go. This should be split into two different functions. I'll see
%%%later
%% Part 1
clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
%map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]; %repeated features
botSim = BotSim(map);  %sets up a botSim object a map, and debug mode on.
resolution = 10;
botSim.setBotPos([10 10]);
botPos = botSim.getBotPos();
botSim.setBotAng(-pi/2);
target = [60 100];
botSim.drawMap();
botSim.drawBot(3);
deltax = target(1)-botPos(1);%second argument would be meanx (or meany)
deltay = target(2)-botPos(2);
if deltax <0 && deltay >0
angledatum = pi+ atan((deltay/deltax));
elseif deltay<0 && deltax<0
    angledatum = pi + atan(deltay/deltax);
elseif deltay ==0 && deltax<0
    angledatum = pi;
else
    angledatum = atan(deltay/deltax);
end

turnangle = angledatum - botSim.getBotAng();%Second argument would be meantheta. Use this to move towards next point n.
scanLines =  [cos(turnangle); sin(turnangle)]'*100;
botSim.setScanConfig(scanLines);
[distance, crossingPoint]  = botSim.ultraScan();
dist2target = sqrt((target(1)-botPos(1))^2 + (target(2)-botPos(2))^2);
botSim.turn(turnangle); %Turn the robot
botSim.move(dist2target);%Move the robot
botSim.getBotAng;

%% Part 2

if distance < dist2target
    fprintf("You can't go in a straight line")
    
else
    % Calculate the nodes along the straight line towards the target
    nodematrix = zeros(2,floor(dist2target/resolution)); %This contains the nodes to go through towards the target
    currnodex = botPos(1);
    currnodey = botPos(2);
    for i = 1:size(nodematrix,2)
       nodematrix(1,i) = currnodex + resolution*cos(botSim.getBotAng);
       nodematrix(2,i) = currnodey + resolution*sin(botSim.getBotAng);
       currnodex = nodematrix(1,i);
       currnodey = nodematrix(2,i);
    end

end

hold off; botSim.drawMap();  %resets drawing area
botSim.drawScanConfig();  %draws the scan configuration to verify it is correct
botSim.drawBot(3);
