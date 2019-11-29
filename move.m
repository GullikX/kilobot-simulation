clear, clc, clf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
individualSizeRadius=15; %[mm]
seeRadius=100; %Not used.
movementSpeed=5;
TurnSpeed=10; %Degrees.

preferedDistance=50; %Bugged at <=2*individualSizeRadius
maxAngleError=10; %Degrees.

dt=1;
timeSteps=80;
populationSize=4+2; %4 immobile + 1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
direction=zeros(populationSize,1); %Degrees.

positions=zeros(populationSize,2);
%Fixed positions.
positions(2,:)=[individualSizeRadius,-sqrt(3)*individualSizeRadius];
positions(3,:)=[2*individualSizeRadius,0];
positions(4,:)=[individualSizeRadius,sqrt(3)*individualSizeRadius];
%Non-fixed positions.
positions(5,:)=[0,-0.5*seeRadius];
positions(6,:)=[0,100];

positionHistory=zeros(timeSteps,2,2); %Remove

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t=1:timeSteps
    positionHistory(t,:,1)=positions(5,:); %Remove
    positionHistory(t,:,2)=positions(6,:); %Remove
    
    %Plot.
	scatter(positions(1:4,1),positions(1:4,2),1000,'g','filled')
    hold on, axis equal
    for i=1:4
        plot(positions(i,1)+preferedDistance*cos(pi/50*(0:100)),positions(i,2)+preferedDistance*sin(pi/50*(0:100)),'c')
    end
    xlim([-150 150])
    ylim([-150 150])
    plot(positionHistory(1:t,1,1),positionHistory(1:t,2,1),'r') %Remove
    plot(positionHistory(1:t,1,2),positionHistory(1:t,2,2),'r') %Remove
    scatter(positions(5:populationSize,1),positions(5:populationSize,2),1000,'b','filled')
    scatter(positions(5:populationSize,1)+individualSizeRadius*cosd(direction(5:populationSize)),positions(5:populationSize,2)+individualSizeRadius*sind(direction(5:populationSize)),'r','filled')
    hold off
    pause(0.1)
    
    for i=5:populationSize
        %Find closest individual.
        nearestIndividual=1;
        nearestDistance=inf;
        for j=1:populationSize
            distance=sqrt(( positions(i,1)-positions(j,1) )^2+( positions(i,2)-positions(j,2) )^2);
            if distance<nearestDistance
                if i~=j
                    nearestDistance=distance;
                    nearestIndividual=j;
                end
            end
        end
        
        %Move.
        dx=positions(i,1)-positions(nearestIndividual,1);
        dy=positions(i,2)-positions(nearestIndividual,2);
        rVector=[dx dy 0];
        w=cross(rVector,[0 0 1]);
        tempVector= w/sqrt(dot(w,w)) + (preferedDistance-sqrt(dx^2+dy^2))/(preferedDistance-2*individualSizeRadius) * rVector/sqrt(dot(rVector,rVector));
        preferedDirectionVector= tempVector/sqrt(dot(tempVector,tempVector));
        
        directionVector=[cosd(direction(i)) sind(direction(i)) 0];
        choiceVector=dot(preferedDirectionVector,directionVector);
        
        if choiceVector<cosd(maxAngleError)
            w=cross(directionVector,[0 0 1]);
            tempVector=dot(w,preferedDirectionVector);
            direction(i)=direction(i) - TurnSpeed * tempVector/sqrt(dot(tempVector,tempVector));
        else
            positions(i,1)=positions(i,1)+movementSpeed*cosd(direction(i))*dt;
            positions(i,2)=positions(i,2)+movementSpeed*sind(direction(i))*dt;
        end
        
        
    end
    
end