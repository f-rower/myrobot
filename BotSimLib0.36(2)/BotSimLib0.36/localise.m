function [botSim,botScan,partData,partLoc,specPart,orientVect] = localise(botSim,map,target)
%This function returns BotSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function
%% setup code: Constants and map
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
nangscans =20; %number of angular scans
%sigma = 5; %standard deviation
maxNumOfIterations = 5;
%% Define the particles. Generate some random particles inside the map. Depends on total size of map
density = 0.02;
num =round((max(map(:,1)) - min(map(:,1)))*(max(map(:,2)) - min(map(:,2)))*density); % number of particles. HOW DO I DETERMINE THIS?
%num = 1;
disp(num);

particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(5); %spawn the particles in random locations. Argument is distance from the walls;
    % particles(i).setBotPos([30 20])
    % particles(i).setBotAng(5*2*pi/nangscans);
    particles(i).setScanConfig(particles(i).generateScanConfig(nangscans));
end
converged =0; %The filter has not converged yet

numBotScans = 20;%How many times we scan at each location
partData = zeros(nangscans+1,num); %Stores particle readings. Last row is for whatever index I use for scoring the particles
botScanCumul = zeros(nangscans,numBotScans); %Stores robot readings
partLoc = zeros(4,num);%Particle location, orientation and weights. Row 1: x, Row 2: y, Row 3: orientation, Row 4: weight
specPart = zeros(6,1);%Stores info about the particles that are above the threshold of probability. Row 1: particle number, R2,3,4: x,y,angle, R5 probability, R6 index value
botScan = zeros(nangscans,1);
saveshift = 0; %Used for turning the particles in the robot's direction once they have converged
%% Localisation and moving code

while converged == 0
[meanx,meany,meantheta] = botloc;
move
end
fprintf("I have localised the robot")
pathplanning
movebottopath
% 
% while Ihaventreachedthetarget
% movebottopath
% moveparticles
% while converged == 0
% botloc
% move
% end
% end

    function [meanx,meany,meantheta]= botloc
    n = 0; % Need to reset n = 0 every time botloc is called
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    for i = 1:numBotScans
    botSim.setScanConfig(botSim.generateScanConfig(nangscans)); %get a scan from the real robot.
    botScanCumul(1:nangscans,i) = botSim.ultraScan();
    end
    for i = 1:nangscans
    botScan(i) = mean(botScanCumul(i,:));
    end
    
    
    %% Write code for updating your particles scans. 
   for i = 1:num
       partData(1:nangscans,i) = particles(i).ultraScan(); %Perform a scan with all particles and store in partData
   end
   for i = 1:num %update the partLoc matrix. MAYBE ONLY GET THE PARTICLE LOCATIONS ONCE WE HAVE RANKED SO THAT I SAVE EXECUTION TIME. LEAVE FOR NOW SO THAT IT'S EASIER TO DEBUG
       loc = particles(i).getBotPos();
       ang = particles(i).getBotAng();
       partLoc(1,i) = loc(1);
       partLoc(2,i) = loc(2);
       partLoc(3,i) = ang;
   end
   
   
    %% Write code for scoring your particles    
    for i = 1:num
        orientVect = zeros(nangscans,1);
        for j = 1:(nangscans) 
            orientVect(j) = sum(abs(botScan(1:nangscans) - circshift(partData(1:nangscans,i),j-1)));%j-1 so that zero shift vector can be evaluated as well.
        end
        [val, loc] = min(orientVect); %Find index of smallest vector element. This will give the number of steps. However, need loc-1 because indices start at 1 and not zero. First element corresponds to zero rotation.
        particles(i).turn(-(loc-1)*2*pi/nangscans) %NO ES JOTA LO QUE TENGO QUE PONER, PORQUE JOTA SIEMPRE VA A SER 30
        partData(size(partData,1),i) = val;
    end
    partLoc(4,:) = abs(partData(size(partData,1),:)-max(partData(size(partData,1),:)));
    sumNorm = sum(partLoc(4,:));
    partLoc(4,:) = partLoc(4,:)/sumNorm;
    wholesum = sum(partLoc(4,:))
    
%% Check for convergence
% meanx = sum((partLoc(1,:)))/size(partLoc,2);
%meany = sum(partLoc(2,:))/size(partLoc,2);
meanx = sum((partLoc(1,:)).*partLoc(4,:));%weighted averages of locations and angle depending on probability
meany = sum((partLoc(2,:)).*partLoc(4,:));
meantheta =mean(partLoc(3,:).*partLoc(4,:));
cumProb = 0; %CONSIDER ONLY RUNNING THIS CODE AFTER THE TWO FIRST ITERATIONS TO SAVE COMPUTING TIME
for i = 1:size(partLoc,2)
    if sqrt((partLoc(1,i)-meanx)^2 + (partLoc(2,i)-meany)^2) <= 1.5
        cumProb = cumProb + partLoc(4,i);
    end
end
cumProb
if cumProb > 0.95
    fprintf ("let's plan the paaaaath")
    converged = 1;
    break
end
    %% Write code for resampling your particles. Here partLoc (4,:) will add up to more than 1.
    ProbMean = mean (partLoc(4,:)); %Find mean of all probabilities
    ProbStd = std (partLoc(4,:));%Find std of all probabilities
    cols = find(partLoc(4,:)>(ProbMean+0.9*ProbStd));%Find the particles whose probability is above the threshold
    if isempty(cols)
      cols = find(partLoc(4,:)>(ProbMean+ProbStd*0.1));  
    end
    s = 0;
    for i = cols
    s = s + partLoc(4,i);
    end
    partLoc(4,cols)= partLoc(4,cols)/s;%Reweight heaviest particles
    %redistribute all particles around these locations according to their
    %weight.
    m = 0;
    lastN = 1;
    rc = 1/n;%radius of circle around which we spawn the particles. Gets smaller with each iteration
    for i = cols %For each particle with heavy weight
        numPart = round(num*partLoc(4,i)); %Calculate the num of particles to be placed around it.
        cumNum = lastN + numPart-1;
        if cumNum > num
            cumNum = num;
        end
        for j = lastN:(cumNum)
            xi = partLoc(1,i);%Get x and y position of heavyweight particle
            yi = partLoc(2,i);
            a=2*pi*rand;%Calculate random position of new particle around it.
            r=sqrt(rand);
            x=(rc*r)*cos(a)+xi;
            y=(rc*r)*sin(a)+yi;
        particles(j).setBotPos([x,y]);
        while ~ botSim.pointInsideMap(particles(j).getBotPos)%If particle is outside map, keep rotating until it falls inside. NECESSARY?
            a = a + pi/3;
            x=(rc*r)*cos(a)+xi;
            y=(rc*r)*sin(a)+yi;
            particles(j).setBotPos([x,y]);    
        end
        m=m+1;
        end
        lastN = m;
    end
    if num>lastN
        remainder = num-lastN;
        for k = lastN: lastN + remainder %Spawn the remaining particles around random locations inside the map.
            particles(k).randomPose(5);
        end
    end
    
    %% Drawing and special particle storage
    %only draw if you are in debug mode or it will be slow during marking
    draw
end
end%Localise robot (position + orientation)
 
function move %Move the robot while not converged
     %% Write code to decide how to move next
     if converged ==0
    [maxdist, index] = max(botScan);
    turn = (index-1)*2*pi/nangscans;
    move = 0.1*maxdist;
    botSim.turn(turn); %turn the real robot.  
    botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
        if particles(i).pointInsideMap(particles(i).getBotPos)==0
            particles(i).randomPose(1)
        end
    end 
    draw
     end
        %only draw if you are in debug mode or it will be slow during marking
end
function pathplanning
    fprintf("I'm planning the path")
    axis equal; %keeps the x and y scale the same
% %map=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];  %default map
% map = [-30,0;-30,40;30,40;30,60;5,60;45,90;85,60;60,60;60,40;120,40;120,60;95,60;135,90;175,60;150,60;150,40;210,40;210,60;185,60;225,90;265,60;240,60;240,40;300,40;300,0]; %repeated features
% %map = [0,0;60,0;60,50;100,50;70,0;110,0;150,80;30,80;30,40;0,80];
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
botLoc = [meanx meany];

% Find closest legal node in the array and set it as the starting node

[~, initloc] = min(sqrt((locArray(1,:)-botLoc(1)).^2 + (locArray(2,:)-botLoc(2)).^2)); %CHANGE ALL THE LOCARRAY STUFF

%Target will be manually set here as well just for testing

%target = [139,69];

% Find closest legal node in the array and set it as the end node

[~, endloc] = min(sqrt((locArray(1,:)-target(1)).^2 + (locArray(2,:)-target(2)).^2));

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
AstarArray(startEnd(1,2),startEnd(2,2),9) = startEnd(1,2) % i index
AstarArray(startEnd(1,2),startEnd(2,2),10) = startEnd(2,2)% j index
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

end 
function movebottopath
    fprintf("I'm moving towards the path");
    
end
% function moveparticles
%     fprintf("I'm moving particles towards the robot");
% end 
function draw
    if botSim.debug()
        figure
        hold off; %the drawMap() function will clear the drawing when hold is off
%         subplot(1,2,1)
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        %botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        botSim.drawBot('g')
       % botSim.drawScanConfig();
        for i =1:num
%             if partLoc(4,i)<0.005
            particles(i).drawParticle(3); %draw particle with line length 3 and default color
%             else
%             particles(i).drawParticle(8,'r'); %draw the special particles in red
%             newcol = zeros(size(specPart,1),1);
%             newcol(1) = i;
%             newcol(2:5) = partLoc(1:4,i);
%             newcol(6) = partData(size(partData,1),i);
%             specPart = [specPart newcol];% on 1st iteration this leaves 1st column as just zeroes
%             end
        end
        drawnow;
%         subplot(1,2,2);
%         plot(1:num,partLoc(4,:))
    end
end
end



