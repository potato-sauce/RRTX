classdef Edge < handle
    %UNTITLED3 此处显示有关此类的摘要
    %   显示详细说明
    
    properties
     startNode;
     endNode;
     dist;
     distOriginal;
    end
    
    methods
        function obj=Edge(a,b)
        obj.startNode=a;
        obj.endNode=b;
        obj.dist=norm(a.position-b.position);
        obj.distOriginal=obj.dist;
        end
    end
    
end

