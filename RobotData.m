classdef RobotData < handle
    %UNTITLED 此处显示有关此类的摘要
    %  此处显示详细说明
    
    properties
        robotPose;
        nextRobotPose;
        nextMoveTarget;
        distanceFromNextRobotPoseToNextMoveTarget
        moving;
        currentMoveInvalid;
        robotMovePath;
        robotLocalPath;
        numRobotMovePoints
        numLocalMovePoints;
        robotEdge;
        robotEdgeUsed;
        distAlongRobotEdge
        robotEdgeForPlottingUsed
        robotEdgeForPlotting
        distAlongRobotEdgeForPlotting
    end
    
    methods
        function obj=RobotData(rP,nMT)
        obj.robotPose = rP;
        obj.nextRobotPose = rP;
        obj.nextMoveTarget = nMT;
        obj.distanceFromNextRobotPoseToNextMoveTarget = 0.0;

        obj.moving = false;
        obj.currentMoveInvalid = false;

        obj.robotMovePath(1,:) = rP;
        obj.numRobotMovePoints = 1;

        obj.robotLocalPath = [];
        obj.robotEdgeUsed = false;
        obj.distAlongRobotEdge = 0.0;
        
        obj.robotEdgeForPlottingUsed = false;
        obj.distAlongRobotEdgeForPlotting = 0.0;
      
        end
    end
    
end

