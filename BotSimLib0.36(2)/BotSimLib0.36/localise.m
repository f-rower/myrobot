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
botloc
move
end
fprintf("I have localised the robot")
% pathplanning
% 
% while Ihaventreachedthetarget
% movebottopath
% moveparticles
% while converged == 0
% botloc
% move
% end
% end

function botloc
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
% function pathplanning
%     fprintf("I'm planning the path")
% end 
% function movebottopath
%     fprintf("I'm moving towards the path");
% end
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



