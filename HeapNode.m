classdef HeapNode < handle
    %UNTITLED2 此处显示有关此类的摘要
    %   显示详细说明
    
    properties
        data;
        heapIndex;
        inHeap;
    end
    
    methods
        function obj=HeapNode(D)
        obj.data=D;
        heapIndex=-1;
        inHeap=false;
        end
    end
    
end

