function [botSim,botScan,partData,partLoc,specPart] = localise(botSim,map,target)
%This function returns BotSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function
%% setup code: Constants and map
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
nangscans =10; %number of angular scans
sigma = 1; %standard deviation
maxNumOfIterations = 1;
%% Define the particles. Generate some random particles inside the map. Depends on total size of map
density = 0.05;
num =round((max(map(:,1)) - min(map(:,1)))*(max(map(:,2)) - min(map(:,2)))*density); % number of particles. HOW DO I DETERMINE THIS?
disp(num);

particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(5); %spawn the particles in random locations. Argument is distance from the walls
    particles(i).setScanConfig(particles(i).generateScanConfig(nangscans));
end

%% Localisation code

n = 0;
converged =0; %The filter has not converged yet


partData = zeros(nangscans+1,num); %Stores particle readings. Last row is for whatever index I use for scoring the particles
botScan = zeros(nangscans+1,1); %Stores robot readings. Last row is for scoring index
partLoc = zeros(4,num);%Particle location, orientation and weights. Row 1: x, Row 2: y, Row 3: orientation, Row 4: weight
specPart = zeros(6,1);%Stores info about the particles that are above the threshold of probability. Row 1: particle number, R2,3,4: x,y,angle, R5 probability, R6 index value
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botSim.setScanConfig(botSim.generateScanConfig(nangscans)); %get a scan from the real robot.
    botScan(1:nangscans) = botSim.ultraScan();
    botScan(size(botScan)) = sum(botScan(1:nangscans));
    %% Write code for updating your particles scans. 
   for i = 1:size(particles)
       partData(1:nangscans,i) = particles(i).ultraScan(); %Perform a scan with all particles and store in partData
   end
   for i = 1:size(particles) %update the partLoc matrix
       loc = particles(i).getBotPos();
       ang = particles(i).getBotAng();
       partLoc(1,i) = loc(1);
       partLoc(2,i) = loc(2);
       partLoc(3,i) = ang;
   end
    %% Write code for scoring your particles    
    for i = 1:size(particles)
       partData(size(partData,1),i) = sum(partData(1:nangscans,i));
    end
   for i = 1:size(particles)
        partLoc(4,i) = (1/sqrt(2*pi*sigma^2))*exp(-(botScan(size(botScan,1))-partData(size(partData,1),i))^2/(2*sigma^2));
   end
    sumNorm = sum(partLoc(4,:));
    partLoc(4,:) = partLoc(4,:)/sumNorm;
    %% Write code for resampling your particles
    
    
    %% Write code to check for convergence   
	

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    
    %% Write code to decide how to move next
    % here they just turn in circles as an example
    turn = 0.5;
    move = 2;
    %botSim.turn(turn); %turn the real robot.  
    %botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing and special particle storage
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        subplot(1,2,1)
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        botSim.drawScanConfig();
        for i =1:num
            if partLoc(4,i)<0.01
            particles(i).drawParticle(3); %draw particle with line length 3 and default color
            else
            particles(i).drawParticle(8,'r'); %draw the special particles in red
            newcol = zeros(size(specPart,1),1);
            newcol(1) = i;
            newcol(2:5) = partLoc(1:4,i);
            newcol(6) = partData(size(partData,1),i);
            specPart = [specPart newcol];% on 1st iteration this leaves 1st column as just zeroes
            end
        end
        drawnow;
        subplot(1,2,2);
        plot(1:num,partLoc(4,:))
    end
end
end
