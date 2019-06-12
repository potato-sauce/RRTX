classdef CSpace < handle
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明
    
    properties
        kd;
        tree;
         hypervolume;%float
        lowerBounds;%array[a,b]
        upperBounds;%array[a,b]
        ob;
        start; %array[a,b]
        goal; %array[a,b]
        width;%array[a,b]
        robotradius;%float
        robotVelocity ;%float
        pGoal;%float;
        delta;%float;
        ballconstant;     %float
        sampleStack;
        goalNode ;
        count;
      root;
       moveGoal;
    startTime;%float
    elapsedTime;%float
    end
         
    methods
        function S=CSpace(envRad,delta,robotradius,robotVelocity,pGoal,ballconstant)
        S.lowerBounds = -envRad*ones(1,2);
       S.upperBounds = envRad*ones(1,2);
       S.width = S.upperBounds-S.lowerBounds;
       S.hypervolume =0.0;
       S.elapsedTime = 0.0;
       S.startTime = 0.0;
       S.delta = delta;
       S.robotradius =robotradius;     
       S.robotVelocity =robotVelocity;     
       S.pGoal=pGoal;
        S.ballconstant=ballconstant;
        S.count=1;
        end
    end
    
end

