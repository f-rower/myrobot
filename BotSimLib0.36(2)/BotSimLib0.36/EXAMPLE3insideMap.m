clf;        %clears figures
clc;        %clears console
clear;      %clears workspace

axis equal; %keeps the x and y scale the same
map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
%map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]; %repeated features
%map = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80];
%% Definition of legal moving locations. Divides map into nodes to which we can move/not move.
botSimPath = BotSim(map);  %sets up a botSim object, a map, and debug mode on.

hold on;
botSimPath.drawMap();

limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
dims = limsMax-limsMin; %dimension of the map
res = 10; %sampling resouloution in cm
iterators = dims/res;
iterators = ceil(iterators)+[1 1]; %to counteract 1 based indexing
AstarArray = zeros(iterators(2),iterators(1),10);
%legalArray = zeros(fliplr(iterators)); %Array to define legal moving locations.
locArray = zeros(10,(size(AstarArray,1)*size(AstarArray,2)));
hold on
%loops through the grid indexes and tests if they are legal moving nodes.
ii=1;
for i = 1:iterators(2) %The array shows the map inverted, but this hopefully simplifies the coding
    for j = 1:iterators(1)
        testPos = limsMin + [j-1 i-1]*res; %to counteract 1 based indexing.RUN THIS ONLY IF THE POINT IS LEGAL
        botSimPath.setBotPos(testPos)
        botScan = botSimPath.ultraScan();
        AstarArray(i,j,1) = botSimPath.pointInsideMap(testPos);
        if AstarArray(i,j,1)
            if isempty(find(botScan==0, 1))&& isempty(find(botScan<10, 1))%Check if point is on map boundary or if it is too close to the wall
                %%%%%%%%%%---------fill
                %%%%%%%%%%AstarArray---------------%%%%%
                
                locArray([1,2],ii) = [testPos(1), testPos(2)];
                locArray([3,4],ii) = [i,j];%Store the matrix indices for this node
                AstarArray(i,j,[4,5]) = [testPos(1),testPos(2)];%Store the matrix indices for this node
                AstarArray(i,j,9) = i;%Store matrix indices at each node
                AstarArray(i,j,10) = j;
                
                plot(testPos(1),testPos(2),'*');%inside map
            else
                AstarArray(i,j,1) = 0;
                plot(testPos(1),testPos(2),'o');%border or too close to border
            end
        else
            plot(testPos(1),testPos(2),'o');%outside map
        end
        ii= ii+1;
    end
end
%% Path finding


startEnd = zeros(2); %This stores the indices of the starting and end nodes.
%Get real bot's location. For testing now, I'll just put in some
%coordinates.
botLoc = [9.081 18.4163];

% Find closest legal node in the array and set it as the starting node

[initdist, initloc] = min(sqrt((locArray(1,:)-botLoc(1)).^2 + (locArray(2,:)-botLoc(2)).^2)); %CHANGE ALL THE LOCARRAY STUFF

%Target will be manually set here as well just for testing

target = [26.5663,72.3859];

% Find closest legal node in the array and set it as the end node

[enddist, endloc] = min(sqrt((locArray(1,:)-target(1)).^2 + (locArray(2,:)-target(2)).^2));

%These are the indices of the starting and end nodes

startEnd(1,1) = locArray(3,initloc); %i index of start node
startEnd(2,1) = locArray(4,initloc); %j index of start node
startEnd(1,2) = locArray(3,endloc);  %i index of end node
startEnd(2,2) = locArray(4,endloc);  %j index of end node

%Path planning algorithm(A*)

%Set 1st point's data
AstarArray(startEnd(1,1),startEnd(2,1),2) = botLoc(1); %Location of robot
AstarArray(startEnd(1,1),startEnd(2,1),3) = botLoc(2);
testPos = limsMin + [startEnd(2,1)-1 startEnd(1,1)-1]*res;
AstarArray(startEnd(1,1),startEnd(2,1),4) = testPos(1);%Location of initial node
AstarArray(startEnd(1,1),startEnd(2,1),5) = testPos(2);
AstarArray(startEnd(1,1),startEnd(2,1),9) = startEnd(1,1); % i index
AstarArray(startEnd(1,1),startEnd(2,1),10) = startEnd(2,1);% j index
% g =AstarArray(startEnd(1,1),startEnd(2,1),6); NOT NEEDED FOR FIRST NODE
% BECAUSE ALL ARE ZERO.
% h =AstarArray(startEnd(1,1),startEnd(2,1),7);
% AstarArray(startEnd(1,1),startEnd(2,1),8) = g+h;

%Set target point's data
%AstarArray(startEnd(1,2),startEnd(2,2),2) = 0; %Location of parent
%AstarArray(startEnd(1,2),startEnd(2,2),3) = 0;
testPos = limsMin + [startEnd(2,2)-1 startEnd(1,2)-1]*res;
AstarArray(startEnd(1,2),startEnd(2,2),4) = testPos(1);%Location of initial node
AstarArray(startEnd(1,2),startEnd(2,2),5) = testPos(2);
AstarArray(startEnd(1,2),startEnd(2,2),9) = startEnd(1,2); % i index
AstarArray(startEnd(1,2),startEnd(2,2),10) = startEnd(2,2);% j index
% g =AstarArray(startEnd(1,2),startEnd(2,2),6); NOT NEEDED FOR TARGET NODE
% BECAUSE ALL ARE ZERO.
% h =AstarArray(startEnd(1,2),startEnd(2,2),7);
% AstarArray(startEnd(1,2),startEnd(2,2),8) = g+h;

%% -------------------A* algorithm------------------------------------------------

curripos =AstarArray(startEnd(1,1),startEnd(2,1),9); %i index of initial node
currjpos =AstarArray(startEnd(1,1),startEnd(2,1),10); %j index of initial node
while 1
    if curripos == startEnd(1,2) && currjpos == startEnd(2,2) %If we are at the goal node, stop searching
        break
    end
    nowhereToGo = zeros(1,6);
    %if AstarArray() CONDITION FOR WHEN NO PATH IS FOUND
    for ii = -1:1
        for jj = -1:1
            if AstarArray(curripos+ii,currjpos+jj,1) == 0 || (ii == 0 && jj == 0)%Do nothing (illegal/closed node or current node)
                %plot(AstarArray(curripos,currjpos,4),AstarArray(curripos,currjpos,5),'s')
            else %If the node is legal
                %--------calculate g(n)---------------------------------
                if ii~=0 && jj~=0 
                    AstarArray(curripos+ii,currjpos+jj,6) = res*1.4;
                else
                    AstarArray(curripos+ii,currjpos+jj,6) = res;
                end
                %------------calculate h(n)
                AstarArray(curripos+ii,currjpos+jj,7) = sqrt((AstarArray(curripos+ii,currjpos+jj,4)-AstarArray(startEnd(1,2),startEnd(2,2),4))^2 +...
                    (AstarArray(curripos+ii,currjpos+jj,5)-AstarArray(startEnd(1,2),startEnd(2,2),5))^2);%Calculate h(n)
                %----------calculate f(n)
                f_n =  AstarArray(curripos+ii,currjpos+jj,6) +  AstarArray(curripos+ii,currjpos+jj,7);%Calculate f(n)
                %-----------check if node already in open list
                if AstarArray(curripos+ii,currjpos+jj,1) == 2 %If already in the open list
                    if AstarArray(curripos+ii,currjpos+jj,8) > f_n  %What if we have the same f cost from two different parents?
                        AstarArray(curripos+ii,currjpos+jj,8) = f_n;
                        AstarArray(curripos+ii,currjpos+jj,2) = curripos;%update parent indices
                        AstarArray(curripos+ii,currjpos+jj,3) = currjpos;
                    else
                        %If the previous was smaller, don't do anything
                    end
                else
                    AstarArray(curripos+ii,currjpos+jj,8) = f_n;%Put f(n) value
                    AstarArray(curripos+ii,currjpos+jj,1) = 2; %Move to open list
                    AstarArray(curripos+ii,currjpos+jj,2) = curripos;
                    AstarArray(curripos+ii,currjpos+jj,3) = currjpos;%set parent indices
                end
            end
        end
    end
    AstarArray(curripos,currjpos,1) = 0;%Send to closed list
    [c ,d] = find(AstarArray(:,:,1)==2);%Find all legal nodes
    minf = 100000;
    for iii = 1:size(c,1) %Find the node with the smallest f cost and set it as the new current node.
        if AstarArray(c(iii),d(iii),8)<minf
            curripos = c(iii);
            currjpos = d(iii);
            minf = AstarArray(c(iii),d(iii),8);
        end
    end
   % plot(AstarArray(curripos,currjpos,4),AstarArray(curripos,currjpos,5),'s')
end

plot(AstarArray(curripos,currjpos,4),AstarArray(curripos,currjpos,5),'s')
while 1
    curriposold = curripos;
    currjposold = currjpos;
    curripos = AstarArray(curriposold,currjposold,2);%Set curripos and currjpos as the parent node
    currjpos = AstarArray(curriposold,currjposold,3);%PROBLEM: HERE I'M USING THE UPDATED CURRIPOS, NOT THE PAST ONE.
    plot(AstarArray(curripos,currjpos,4),AstarArray(curripos,currjpos,5),'s')
    if curripos == startEnd(1,1) && currjpos == startEnd(2,1)
        break
    end
end

