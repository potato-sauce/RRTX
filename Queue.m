classdef Queue < handle
    %UNTITLED 此处显示有关此类的摘要
    %   显示详细说明
    
    properties
        OS;
        Q;
        S;
        changeThresh;
    end
    
    methods
        function obj=Queue(changeThresh,S)
            obj.Q=BinaryHeap();
            obj.S=S;
            obj.changeThresh=changeThresh;
            
        end
    end
    
end

