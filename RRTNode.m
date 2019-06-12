classdef RRTNode < handle
    %UNTITLED2 此处显示有关此类的摘要
    %   显示详细说明
    
    properties
       kdInTree;
        rrtParentUsed ; 
        inOSQueue;
        isMoveGoal ;
        
        inPriorityQueue;
        priorityQueueIndex;
      
        rrtTreeCost ;
        rrtLMC ;
        position;
        neighborsOut;
        neighborsIn ;
        InitialNeighborListOut;
        InitialNeighborListIn;
        SuccessorList ; 
        rrtParentEdge;
            tempEdge;
            
    end
    
    methods
        function obj= RRTNode(position)
        obj.kdInTree = false;
        obj.rrtParentUsed = false ; 
        obj.isMoveGoal =false ;
       
        obj.inOSQueue=false;
        obj.inPriorityQueue=false;
        obj.priorityQueueIndex=-1;
        
        obj.rrtTreeCost = Inf;
        obj.rrtLMC = Inf;
        obj.position=position;
        obj.neighborsOut = [];
        obj.neighborsIn = [];
        obj.InitialNeighborListOut=[];
        obj.InitialNeighborListIn=[];
        obj.SuccessorList = []; 
        obj.rrtParentEdge = [];
        obj.tempEdge=[];
        end
        
        
    end
    
end

