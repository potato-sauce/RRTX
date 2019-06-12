classdef BinaryHeap < handle
    %UNTITLED 此处显示有关此类的摘要
    %   显示详细说明
    
    properties
        heapNode;
        indexOfLast;
        parentOfLast;
    end
    
    methods
        function obj=BinaryHeap()
        obj.heapNode=[];
        obj.indexOfLast=0;
        obj.parentOfLast=-1;
        end
    end
    
end

