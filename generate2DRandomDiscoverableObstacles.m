

outputFile = './environments/rand_Disc2.txt';       % file to save

environmentSpans = [-50, 50, -50, 50];           % [xmin, xmax, ymin, ymax]

numObstacles = 65;                              % number of obstacles
lifeSpanBounds = [2 10];                        % in seconds

probNormal = 0.1;                               % probability the obstacle is normal (always "on")

probNonNormalVanish = 0.5;                      % if the obstacle is not normal, then with this probability
% it starts as an obstacle and then vanishes, and with,
% 1 - probNonNormalVanish it starts hidden and then shows
% up when the robot gets close to it.


maxPointsPerPolygon = 10;                       % paramiters for generating a polygon obstacle
minPointsPerPolygon = 3;
minPolygonRadius = 5;
maxPolygonRadius = 15;

noObsPad = -10;                                 % do not place obstacles closer to the border than this plus maxPolygonRadius


startPoint = [20, 30];                          % will not put obstacle here
goalPoint = [0, -40];                           % will not put obstacle here



padPlusMaxRad = noObsPad+maxPolygonRadius;
environmentSpansWithPad = environmentSpans - [-padPlusMaxRad, padPlusMaxRad, -padPlusMaxRad, padPlusMaxRad];


fileID = fopen(outputFile,'w');
fprintf(fileID,'%d\n', numObstacles);
for n = 1:numObstacles
    MaxRadius = minPolygonRadius + rand(1,1)*(maxPolygonRadius - minPolygonRadius);
    numPoints = minPointsPerPolygon+ floor(rand(1,1)*(maxPointsPerPolygon-minPointsPerPolygon));
    
    % define a loose center point
    distToGoal = 0;
    distToStart = 0;
    centerPoint = [environmentSpansWithPad(1), environmentSpansWithPad(3)] + rand(1,2).*[environmentSpansWithPad(2) - environmentSpansWithPad(1), environmentSpansWithPad(4) - environmentSpansWithPad(3)];
    while sqrt(sum((centerPoint - startPoint).^2)) - MaxRadius <= 0 ...
          || sqrt(sum((centerPoint - goalPoint).^2)) - MaxRadius <= 0
      centerPoint = [environmentSpansWithPad(1), environmentSpansWithPad(3)] + rand(1,2).*[environmentSpansWithPad(2) - environmentSpansWithPad(1), environmentSpansWithPad(4) - environmentSpansWithPad(3)];
    end
    
    % first pick numPoints angles, these will define the angle of a ray
    % emminating from -environmentSpan environmentSpan -environmentSpan environmentSpancenterPoint
    rayAngles = sort(rand(numPoints,1)*2*pi);
    
    % now randomly place the polygon's points a rand value between 0 and MaxRadius
    % along each ray
    distances = sqrt(rand(numPoints,1))*MaxRadius;
    
    pointsX{n} = distances.*cos(rayAngles) + centerPoint(1);
    pointsY{n} = distances.*sin(rayAngles) + centerPoint(2);
    
    A = [pointsX{n}, pointsY{n}]';
    fprintf(fileID,'%d\n', numPoints);
    fprintf(fileID,'%f, %f\n', A);
    
    if rand() <= probNormal
        obsBehaviourType{n} = 0;
    elseif rand() <= probNonNormalVanish
        obsBehaviourType{n} = -1;
    else
        obsBehaviourType{n} = 1;
    end
    fprintf(fileID,'%d\n', obsBehaviourType{n});
end

fclose(fileID);

figure(1)
if obsBehaviourType{1}  == 0
    pltclr = 'k';
elseif obsBehaviourType{1}  == -1
    pltclr = 'g';
elseif obsBehaviourType{1} == 1
    pltclr = 'r';
end
plot([pointsX{1}; pointsX{1}(1)], [pointsY{1}; pointsY{1}(1)], pltclr)
axis(environmentSpans)
hold on
for n = 2:numObstacles
    
    if obsBehaviourType{n}  == 0
        pltclr = 'k';
    elseif obsBehaviourType{n}  == -1
        pltclr = 'g';
    elseif obsBehaviourType{n} == 1
        pltclr = 'r';
    end
    
    plot([pointsX{n}; pointsX{n}(1)], [pointsY{n}; pointsY{n}(1)], pltclr)
end
hold off

