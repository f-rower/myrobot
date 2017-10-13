function [botSim,partData] = localise(botSim,map,target)
%This function returns BotSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
nangscans =20; %number of angular scans
%generate some random particles inside the map. Depends on total size of
%the map
density = 0.03;
num =round((max(map(:,1)) - min(map(:,1)))*(max(map(:,2)) - min(map(:,2)))*density); % number of particles. HOW DO I DETERMINE THIS?
disp(num);
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(5); %spawn the particles in random locations. Argument is distance from the walls
    particles(i).setScanConfig(particles(i).generateScanConfig(nangscans));
end

%% Localisation code
maxNumOfIterations = 1;
n = 0;
converged =0; %The filter has not converged yet
partData = zeros(nangscans+1,num); %Stores particle readings. Last row is for whatever index I use for scoring the particles
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1; %increment the current number of iterations
    botSim.setScanConfig(botSim.generateScanConfig(nangscans)); %get a scan from the real robot.
    botScan = botSim.ultraScan();
    disp(botScan)
    %% Write code for updating your particles scans
   for i = 1:size(particles)
       partData(1:nangscans,i) = particles(i).ultraScan();
   end
    
    %% Write code for scoring your particles    
    
    
    %% Write code for resampling your particles
    
    
    %% Write code to check for convergence   
	

    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    
    %% Write code to decide how to move next
    % here they just turn in cicles as an example
    turn = 0.5;
    move = 2;
    %botSim.turn(turn); %turn the real robot.  
    %botSim.move(move); %move the real robot. These movements are recorded for marking 
    for i =1:num %for all the particles. 
        particles(i).turn(turn); %turn the particle in the same way as the real robot
        particles(i).move(move); %move the particle in the same way as the real robot
    end
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
        botSim.drawScanConfig();
        for i =1:num
            particles(i).drawParticle(3); %draw particle with line length 3 and default color
        end
        drawnow;
    end
end
end
