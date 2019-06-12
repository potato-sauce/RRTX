function RRT()
        
%%%%%%%%%%%%%%%%%%%%initialize the necessary parameters %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       changeThresh =2.0;
       slice_time =2;%used for recaculatin after the robot has moved
       robotSensorRange=20.0;%the robot's Sensor
       envRad=50.0;
      robotradius = 0.53;%the physical robot model
       robotVelocity =2.5;% the length of the robot go forward each slice time
       pGoal = 0.1;%possibility of tending to S.root
       delta = 5.0;%granularity of nodes for generating 
       ballconstant=100.0;%parameter of generating the detective range of culling
       S=CSpace(envRad,delta,robotradius,robotVelocity,pGoal,ballconstant);
       
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %%%%%%%%%%choose the environments%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       %rand_Disc_grid_:static obstacles
       %rand_Disc_grid_1:dynamic obstacles appear
       %rand_Disc_grid_2:static without obstacles
       S.ob=readObstacle('environments\rand_Disc_grid_1.txt');%obtain the obstacle data
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
       S.start = [40.0 -30.0];  %robot goes to here (start location of search tree)
       S.goal = [-30.0 40.0];  % robot comes from here (goal location of search tree)
      
       
       
    
      
       total_planning_time=50.0;%the time use for planning the tree before the robot starts moving  
       T=clock;
      % startTime=etime(clock,T);
       save_elapsed_time=0.0;
      
       
       S.delta = delta;
       
       %%%keep nodes which are influented by the obstacle
       Q=Queue(changeThresh,S);
       
       
       root=RRTNode(S.start);
       explicitlyUnSafe=explicitNodeCheck(S,root.position);
       if explicitlyUnSafe
        error('root is not safe');
       end
       root.rrtTreeCost=0.0;
       root.rrtLMC=0.0; 
       
   
        S.kd=repmat([NaN,NaN],6000,1);
       S.kd(S.count,:)=root.position;
       root.kdInTree=true;
       S.tree=repmat(RRTNode([NaN,NaN]),6000,1);
       S.tree(S.count,:)=root;
       S.count=S.count+1;
       
       goal=RRTNode(S.goal);
       goal.rrtTreeCost = Inf;
       goal.rrtLMC = Inf;
       
       S.goalNode = goal;
       S.root=root;
       S.moveGoal=goal;
       S.moveGoal.isMoveGoal =true;
       
       R=RobotData(S.goal,goal);%save moving data of the sumulate robot 
       
       vCounter = 0;%helps with visuilizing data
       sliceCounter = 0;
       S.startTime = etime(clock,T); 
       S.elapsedTime = 0.0;
%%%%%%%%%%%%%%%%%%%%end of the initializing parameters%%%%%%%%%%%%%%%%%%%%%%%%%%      
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
       while true
           
           hyberBallRad=min(delta,ballconstant*((log(1+S.count)/(S.count))^(1/2)));%the detective range of robot for neighbors
           slice_end_time = (1+sliceCounter)*slice_time;
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           %%%%%%%%add/remove newly "detected" obstacles %%%%%%%%%%
           %%%%%%%%add/remove newly "detected" obstacles %%%%%%%%%%
           %%%%%%%%add/remove newly "detected" obstacles %%%%%%%%%%
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           %%%%%beginning of remove obstacle%%%%%%%%%%%%%%%%%%%%%%%%
          %%%%%%%%%% %remove obstacles at the required time
          S.elapsedTime=etime(clock,T)-S.startTime-save_elapsed_time;
           removedObstacle = false;     
           for i=1:size(S.ob,2)
            obs=S.ob(i);     
              if obs.senseableObstacle&&obs.obstacleUnusedAfterSense...
                       &&norm(R.robotPose-obs.position)< robotSensorRange + obs.radius
    
       
                            randomSampleObs(S,obs); %stores samples in the sample stack
                            removeObstacle(S, Q, obs, root, hyberBallRad, S.moveGoal);
                            obs.senseableObstacle = false;
                            obs.startTime = Inf;
                            removedObstacle = true;
               end
           end
          %if it has been changed,we have to update the local minimum
          %cost of the nodes that crash into the obstacle 
           if removedObstacle
               %propogates cost information through the graph
                  reduceInconsistency(Q, S.moveGoal,root, hyberBallRad);
           
           end
      
          %%%%%%%%%%%%%%%%%%%%%%end of remove obstacle%%%%
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          %%%%%%%%%%beginning of add obstacle%%%%%%%%%%
         
           addedObstacle=false;
           for i=1:size(S.ob,2)
           obs=S.ob(i);
           if ~obs.senseableObstacle && obs.obstacleUnused && (obs.startTime <= S.elapsedTime &&S.elapsedTime<= obs.startTime +obs.lifespan)
                    %time to add
                    addNewObstacle(S,Q, obs, R);
                    addedObstacle = true;
                    
           else
               if obs.senseableObstacle && ~obs.obstacleUnusedAfterSense && norm(R.robotPose-obs.position) < robotSensorRange + obs.radius
                     % place to add obstacle
                     addNewObstacle(S,Q, obs, R);
                     obs.senseableObstacle = false;
                     addedObstacle = true;
           
               end
           end
           end
           if addedObstacle
                   %propogate inf cost to all nodes beyond the obstacle 
                   propogateDescendants(Q, R);                  
                   if ~S.moveGoal.inOSQueue
                        verifyInQueue(Q, S.moveGoal);
                   end               
                   reduceInconsistency(Q, S.moveGoal,root, hyberBallRad);
           end
            
           
           %%%%%%%%%%%%%%end of add obstacle%%%%%%%%%%%%%%%%%%%%%%%%%%
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
           %%%%%%%done with add/remove newly "detected" obstacles%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
          
           S.elapsedTime=etime(clock,T)-S.startTime- save_elapsed_time;
           if S.elapsedTime >= slice_end_time
                %calculate the end time of the next slice
                slice_end_time = (1+sliceCounter)*slice_time;
               
                sliceCounter=sliceCounter+1;
                truncElapsedTime = floor(S.elapsedTime * 1000)/1000;
                fprintf('%f-----%f  %f---\n',truncElapsedTime,S.moveGoal.rrtTreeCost,S.moveGoal.rrtLMC);
               
                
                %move robot if the robot is allowed to move, otherwise planning is finished
                %so break out of the control loop
                if S.elapsedTime > total_planning_time + slice_time
                   %save the root path data
                   %print the current position of the robot
                        moveRobot(S,slice_time, root, hyberBallRad, R);
                    
                end
                
                reduceInconsistency(Q, S.moveGoal,root, hyberBallRad);
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
                %%%%%%%%%%%%%%%%%%%%%%%%%%%visualize graph%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
         
                before_save_time=etime(clock,T);
                saveRRTTree(S,['temp/edges_' num2str(vCounter) '.txt']);
                saveRRTNodes(S,['temp/nodes_' num2str(vCounter) '.txt']);    
                saveRRTPath(S.moveGoal, root, R, ['temp/path_' num2str(vCounter) '.txt']);
                saveObstacleLocations(S.ob,['temp/obstacles_' num2str(vCounter) '.txt']);
                saveData(R.robotMovePath(1:R.numRobotMovePoints,:),['temp/robotMovePath_' num2str(vCounter) '.txt']); 
                vCounter =vCounter + 1;
                save_elapsed_time =save_elapsed_time + etime(clock,T)-before_save_time;
                
                %%%end of visualize graph%%%
                
                %check if the robot has reached its movement goal
                if R.robotPose == root.position 
                    display('finish');
                    break;
                end
           end
           %%%end of obstacle and robot pose update
           
           %%%%%%%%%start of normal graph search stuff%%%%%%%%%
           
           %pick a random node to expand the tree   
            newNode =randNodeOrFromStack(S);
            
            if newNode.kdInTree
                continue;
            end
            [closestNode, closestDist] = kdFindNearest(S, newNode.position);
            if closestDist > delta && newNode ~= S.goalNode
                newNode.position=closestNode.position+(newNode.position - closestNode.position)*delta/closestDist;
            end
            explicitlyUnSafe=explicitNodeCheck(S, newNode.position);         
             if explicitlyUnSafe
                    continue;
             end
                
     
            extend(S,Q, newNode, closestNode,hyberBallRad, S.moveGoal);
       
            reduceInconsistency(Q, S.moveGoal,root, hyberBallRad);
       
       end
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%kd functions%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function new=randNodeOrFromStack(S)
    
if size(S.sampleStack,1)>0
        new=RRTNode(S.sampleStack(1,:));
        S.sampleStack(1,:)=[];
    else
        if rand()>S.pGoal
            new=RRTNode(S.lowerBounds + ( rand(1, 2) .* S.width ));
        else
            new=S.goalNode;
        end
    end
end
function [closestNode,closestDist]=kdFindNearest(S, newNodeposition)
        
        datarow=S.count;
        diffMat=repmat(newNodeposition,[datarow,1])-S.kd(1:S.count,:);
        distanceMat=sqrt(sum(diffMat.^2,2));
        closestDist=min(distanceMat);
        Ind= find(distanceMat==closestDist);
        if size(Ind,1)~=1
        Ind=Ind(1);
        end
        closestNode=S.tree(Ind,:); 
end
function index=kd_find_range(S,range,querypoint)
    datarow=S.count;
    diffMat=repmat(querypoint,[datarow,1])-S.kd(1:S.count,:);
    distanceMat=sqrt(sum(diffMat.^2,2));
    index=find(distanceMat<=range);
    
end
%%%%%%%%%%%%end of kd function %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%%%generate a new Node for planning%%%%%%%%%%%%%%%%%%%%%%%%
function randomSampleObs(S,ob)

    obHypervolumeBound=(2.0*ob.radius)^2;
    
    if S.hypervolume == 0.0
        S.hypervolume=prod(S.width);
    end
    numObsSamples = S.count* obHypervolumeBound/S.hypervolume + 1.0;
        
        for smp =numObsSamples:-1.0:-1.0
            
            newPoint =rand(1,2);
            newPoint(1:2) = ob.position(1:2)- ob.radius + newPoint(1:2) * ob.radius*2.0;
            
            if quickCheck2D(ob, newPoint)
               S.sampleStack=[S.sampleStack;newPoint] ;
            end
        end
    

end
%%%%%%%%%%%%%end of generating %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%handle the obstacles%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function addNewObstacle(S,Q,ob,R)
   
    ob.obstacleUnused = false;
    Inds= findPointsInConflictWithObstacle(S,ob);
    
    Nodes=S.tree(Inds,:);
    m=size(Nodes,1);
    for i=m:-1:1
        thisNode=Nodes(i);
        thisNodeOutInitialNeighbors= thisNode.InitialNeighborListOut;
        thisNodeOutNeighbors=thisNode.neighborsOut;
        tempNeighbors=[thisNodeOutInitialNeighbors,thisNodeOutNeighbors];
        n=size(tempNeighbors,2);
        for j=n:-1:1
            
            if explicitEdge3Check(ob,tempNeighbors(j),S.robotradius)
                tempNeighbors(j).dist=Inf;
            end
        end
        if thisNode.rrtParentUsed && ...
                explicitEdge3Check(ob, thisNode.rrtParentEdge,S.robotradius)
            tempInd=size(thisNode.rrtParentEdge.endNode.SuccessorList,2);
            for j=tempInd:-1:1
                if thisNode.rrtParentEdge.endNode.SuccessorList(j)==thisNode
                    thisNode.rrtParentEdge.endNode.SuccessorList(j)=[];
                    break;
                end
            end
            thisNode.rrtParentEdge.endNode = thisNode;
            thisNode.rrtParentEdge.dist = Inf;
            thisNode.rrtParentUsed = false;
            verifyInOSQueue(Q, thisNode)
        end
         
    end
    L=[];
    if R.robotEdgeUsed && explicitEdge3Check(ob, R.robotEdge,S.robotradius)
        R.currentMoveInvalid = true;
    end
end
function removeObstacle(S,Q, ob, root, hyberBallRad, moveGoal)
    
    Inds= findPointsInConflictWithObstacle(S, ob);
    L=S.tree(Inds,:);
    
    for i=size(L,1):-1:1
        thisNode=L(i);
          neighborsWereBlocked=false;
          
          
          thisNodeOutInitialNeighbors= thisNode.InitialNeighborListOut;
          thisNodeOutNeighbors=thisNode.neighborsOut;
          tempNeighbors=[thisNodeOutInitialNeighbors,thisNodeOutNeighbors];
          s=size(tempNeighbors,2);
          if s==0
             
          else
          for num=s:-1:1
            if tempNeighbors(num).dist==Inf...
                  &&explicitEdge3Check(ob,tempNeighbors(num),S.robotradius)
              %conflict with another obstacle
              conflictsWithOtherObs = false;
              
              for j=size(S.ob,2):-1:1
                  obOther=S.ob(j);
                 if obOther~=ob && ~obOther.obstacleUnused 
                        if explicitEdge3Check(obOther,tempNeighbors(num),S.robotradius)
                            conflictsWithOtherObs = true;
                                break;
                        end
                 end
              end
              
              if ~conflictsWithOtherObs
                  tempNeighbors(num).dist=tempNeighbors(num).distOriginal;
                  neighborsWereBlocked = true;
              end
            end
          end
          
          if neighborsWereBlocked
              recalculateLMCMineVTwo(thisNode, root, hyberBallRad);
                if thisNode.rrtTreeCost ~= thisNode.rrtLMC && lessQ(thisNode, moveGoal)
                verifyInQueue(Q, thisNode)
                end
          end
          end
       
   end
   L=[];
   ob.obstacleUnused = true;
end
function L=findPointsInConflictWithObstacle(S,ob)
searchRange = S.robotradius + S.delta + ob.radius;
L=kd_find_range(S,searchRange,ob.position);
end
%%%%%%%%%%%%end of handling the obstacle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%robot moving%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function moveRobot(S,slice_time,root,hyberBallRad,R)

%start by updating the location of the robot based on how it moved
%since the last update (as well as the total path that it has followed)
    if R.moving
         R.robotPose = R.nextRobotPose;
         R.robotMovePath(R.numRobotMovePoints+1:R.numRobotMovePoints+R.numLocalMovePoints,:) = R.robotLocalPath(1:R.numLocalMovePoints,:);
         R.numRobotMovePoints = R.numRobotMovePoints+R.numLocalMovePoints;
         
         fprintf('new robot pose: [%f %f]\n',R.robotPose(1),R.robotPose(2));
         
        %save stuff for plotting
         R.distAlongRobotEdgeForPlotting = R.distAlongRobotEdge;
         
         R.robotEdgeForPlotting = R.robotEdge;
         R.robotEdgeForPlottingUsed = true;
    else
        %movement has just started, so remember that the robot is now moving
        R.moving=true;
        
        if ~S.moveGoal.rrtParentUsed
            %no parent has been found for the node at the robot's position
             R.currentMoveInvalid = true;
        else
             R.robotEdge = S.moveGoal.rrtParentEdge;
             R.robotEdgeForPlotting = R.robotEdge;
             R.robotEdgeUsed = true;
             R.robotEdgeForPlottingUsed = true;
             
             R.distAlongRobotEdge = 0.0;
             R.distAlongRobotEdgeForPlotting = 0.0;
      
        end
    end
%if the robot's current move target has been invalidated due to 
 %dynamic obstacles then we need to attempt to find a new (safe) move
 
 %if the robot's current move target has been invalidated due to 
 %dynamic obstacles then we need to attempt to find a new (safe) move target
    if R.currentMoveInvalid
        
    findNewTarget(S,R, hyberBallRad);
    
    
    else
        S.moveGoal.isMoveGoal = false;
        S.moveGoal = R.nextMoveTarget;
        S.moveGoal.isMoveGoal = true;
    end
    
    
  %finally, we calculate the point to which the robot will move to in slice_time
  %and remember it for the next time this function is called. we also remember
  %all the nodes that it will visit along the way in the local path
  %and the part of the edge trajectory that takes the robot to the first local
  %point (the latter two things are used for visuialization)
   nextNode = R.nextMoveTarget;
   
   
%calculate distance from robot to the end of the current edge it is following
   nextDist = R.robotEdge.dist - R.distAlongRobotEdge;
   
   
  %dist left for robot to move 
   distRemaining = S.robotVelocity*slice_time ;
   
   
   %save first local path point
    R.numLocalMovePoints = 1;
    R.robotLocalPath(R.numLocalMovePoints,:) = R.robotPose;
    
    
    
    %starting at current location (and looking ahead to nextNode), follow parent 
    %pointers back for the approperiate distance (or root or dead end)
    while (nextDist <= distRemaining && nextNode ~= root &&...
           nextNode.rrtParentUsed && nextNode ~= nextNode.rrtParentEdge.endNode)
       %can go all the way to nextNode and still have some distance left to spare
       %remember the robot will move through this point
      R.numLocalMovePoints =R.numLocalMovePoints+ 1;
      R.robotLocalPath(R.numLocalMovePoints,:) = nextNode.position ;
      %recalculate remaining distance
      distRemaining =distRemaining - nextDist;

      %reset distance along edge
      R.distAlongRobotEdge = 0.0;

      %update trajectory that the robot will be in the middle of
      R.robotEdge = nextNode.rrtParentEdge;
      R.robotEdgeUsed = true;

      %calculate the dist of that trajectory
      nextDist = R.robotEdge.dist;

      %update the next node (at the end of that trajectory)
      nextNode = R.robotEdge.endNode;

    end
    
    
    
    %calculate next pose of the robot
    if nextDist > distRemaining
      R.distAlongRobotEdge = R.distAlongRobotEdge+distRemaining;
      if R.robotEdge.dist==0.0
        R.nextRobotPose=R.robotEdge.endNode.position;
      else
      ratioAlongEdge= R.distAlongRobotEdge/R.robotEdge.dist;
       R.nextRobotPose=R.robotEdge.startNode.position + ratioAlongEdge*(R.robotEdge.endNode.position - R.robotEdge.startNode.position);
      
       end
    else
      %the next node is the end of this tree and we reach it
      R.nextRobotPose = nextNode.position;
      R.distAlongRobotEdge = R.robotEdge.dist;
    end

    R.nextMoveTarget = R.robotEdge.endNode;

    %remember last point in local path
    R.numLocalMovePoints =R.numLocalMovePoints+ 1;
    R.robotLocalPath(R.numLocalMovePoints,:) = R.nextRobotPose;

end
%attempts to find a new move target for the robot, places it into Robot data
%(used when the old target has become invalid)
function findNewTarget(S,R,hyberBallRad)
%  start by looking at a hyperball of possible targets with a radius
%  determined by max of {hyberBallRad} and {the previous edge
% legth (the edge that is now invalud)}. If this fails, then we try larger
%  and larger balls until we have looked at all nodes, if still cannot
%  find a target then the robot is out of luck, and we do nothing
  R.robotEdgeUsed = false;
  R.distAlongRobotEdge = 0.0;
  R.robotEdgeForPlottingUsed = false;
  R.distAlongRobotEdgeForPlotting = 0.0;
  display('moving target is invalid');
  
  
  searchBallRad = max(hyberBallRad, norm(R.robotPose-R.nextMoveTarget.position));
  maxSearchBallRad = norm(S.lowerBounds - S.upperBounds);
  searchBallRad = min(searchBallRad, maxSearchBallRad);
  L = kd_find_range(S, searchBallRad, R.robotPose);
 
  nodes=S.tree(L,:);
  dummyRobotNode = RRTNode(R.robotPose); %a temp node at robot pose
while true
    
    fprintf('searching for new target--radius %f\n',searchBallRad);
    
    bestDistToNeighbor = Inf;
    bestDistToGoal = Inf;
    bestNeighbor =[];
    m=size(nodes,1);
    for i=1:m
        neighborNode=nodes(i);
        thisEdge =Edge(dummyRobotNode, neighborNode);
        
        if ~explicitEdge2Check(S, thisEdge)
            %a safe point was found, see if it is the best so far
            distToGoal = neighborNode.rrtLMC + thisEdge.dist;
                if distToGoal < bestDistToGoal
                    bestDistToGoal = distToGoal;
                    bestDistToNeighbor = thisEdge.dist;
                    bestNeighbor = neighborNode;
                    edgeToBestNeighbor = thisEdge;
                end
        end
    end
    %if a valid neighbor was found, then use it
    if bestDistToGoal ~= Inf
      R.nextMoveTarget = bestNeighbor;
      R.distanceFromNextRobotPoseToNextMoveTarget = bestDistToNeighbor;
      R.currentMoveInvalid = false;
      display('found a valid move target');
      R.robotEdge = edgeToBestNeighbor;
      R.robotEdgeForPlotting = edgeToBestNeighbor;
      R.robotEdgeUsed = true;
      R.robotEdgeForPlottingUsed = true;
      %note this is updated before robot moves
      
      R.distAlongRobotEdge = 0.0 ;           
      R.distAlongRobotEdgeForPlotting = 0.0 ;
      
      S.moveGoal.isMoveGoal = false;
      S.moveGoal = R.nextMoveTarget;
      S.moveGoal.isMoveGoal = true;
      
      break;
    end
    searchBallRad=searchBallRad*2;
    if searchBallRad > maxSearchBallRad*2
      error('unable to find a valid move target');
    end
    
    L=kd_find_range(S, searchBallRad, R.robotPose);
    nodes=S.tree(L,:);
end

L=[];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%end of moving %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%the functions of saving data%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function saveRRTTree(S,filename)
   t=[];
    
    m=size(S.tree,1);
    for i=m:-1:1
        if S.tree(i).rrtParentUsed
            t=[t;S.tree(i).position,S.tree(i).rrtTreeCost;S.tree(i).rrtParentEdge.endNode.position,S.tree(i).rrtParentEdge.endNode.rrtTreeCost];
           
        end
    end
   dlmwrite(filename,t,'-append');
end
function saveRRTNodes(S,filename)
    
    cost=[S.tree(1:S.count,:).rrtTreeCost]';
    LMC=[S.tree(1:S.count,:).rrtLMC]';
    data=[S.kd(1:S.count,:),cost,LMC];
    dlmwrite(filename,data);
end
function saveRRTPath(node,root,robot,filename)
    t=[];
    thisNode = node;
    if robot.robotEdgeForPlottingUsed && robot.robotEdgeForPlotting.startNode ~= thisNode
    ratioAlongEdge=robot.distAlongRobotEdgeForPlotting/robot.robotEdgeForPlotting.dist;
    ret = robot.robotEdgeForPlotting.startNode.position + ratioAlongEdge*(robot.robotEdgeForPlotting.endNode.position - robot.robotEdgeForPlotting.startNode.position);
    t=[t;ret;robot.robotEdgeForPlotting.endNode.position];
   
    end
    
    i = 0; %added to detect for inf paths
    while thisNode ~= root && thisNode.rrtParentUsed && i < 1000
    
    t=[t;thisNode.rrtParentEdge.startNode.position;thisNode.rrtParentEdge.endNode.position];
   
    thisNode = thisNode.rrtParentEdge.endNode;
    i = i + 1; % added to detect for inf paths
    end
    t=[t;thisNode.position];
    dlmwrite(filename,t,'-append');
end
function saveObstacleLocations(obstacles, filename)
    t=[];
    m=size(obstacles,2);
    for i=1:m
    
        if obstacles(i).obstacleUnused
            continue;
        end
        
        
       t=[t;obstacles(i).polygon;obstacles(i).polygon(1,:);[NaN,NaN]];
    end
     dlmwrite(filename,t,'-append');
end
function saveData(data, filename)
    dlmwrite(filename, data,'-append');
end
%%%%%%%%%%%%end of the functions of saving data%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%RRTX main operations%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%recalculates LMC based on neighbors
function recalculateLMCMineVTwo(thisNode, root, hyberBallRad)
    if thisNode == root
    return;
    end
    
      newParentFound = false;
      rrtParent = [];
      rrtParentEdge = [];
      cullCurrentNeighbors(thisNode, hyberBallRad);
      thisNodeOutInitialNeighbors=thisNode.InitialNeighborListOut;
          thisNodeOutNeighbors=thisNode.neighborsOut;
          tempNeighbors=[thisNodeOutInitialNeighbors,thisNodeOutNeighbors];
      for i=1:size(tempNeighbors)
          neighborEdge = tempNeighbors(i);
          neighborNode = neighborEdge.endNode;
          neighborDist = neighborEdge.dist;
          if neighborNode.inOSQueue
              continue;
          end
          if (thisNode.rrtLMC > neighborNode.rrtLMC + neighborDist &&...
        (~neighborNode.rrtParentUsed ||...
         neighborNode.rrtParentEdge.endNode ~= thisNode))
            thisNode.rrtLMC = neighborNode.rrtLMC + neighborDist;
            rrtParent = neighborNode;
            rrtParentEdge = neighborEdge;
            newParentFound = true;
          end
          
          if newParentFound
              makeParentOf(rrtParent, thisNode, rrtParentEdge);
          end
      end
end
%takes care of inserting a new node
function extend(S,Q,newNode,closestNode,hyberBallRad,moveGoal)
    
    nodeIndex = kd_find_range(S, hyberBallRad, newNode.position);
    nodeList=S.tree(nodeIndex,:);
    [newNode,nodeList]=findBestParent(S, newNode, nodeList, closestNode);
    if ~newNode.rrtParentUsed
    nodeList=[];    %clean up
    return;
    end
    
   parentNode = newNode.rrtParentEdge.endNode;
   backEdge = Edge(parentNode, newNode);
   backEdge.dist = Inf;
   parentNode.SuccessorList=[parentNode.SuccessorList,backEdge];
    S.tree(S.count,1)=newNode;
    S.kd(S.count,:)=newNode.position;
    newNode.kdInTree=true;
    S.count=S.count+1;
    
    m=size(nodeList,1);
    for i=1:m
        nearNode=nodeList(i,:);
        if norm(nearNode.position-newNode.position)~=Inf
        newNode.InitialNeighborListOut=[newNode.InitialNeighborListOut,nearNode.tempEdge];
        newNode.neighborsOut=[newNode.neighborsOut,nearNode.tempEdge];
        nearNode.neighborsIn=[nearNode.neighborsIn,nearNode.tempEdge];
        end
        
        thisEdge=Edge(nearNode,newNode);
        
        if ~explicitEdge2Check(S, thisEdge)
           nearNode.InitialNeighborListIn= [nearNode.InitialNeighborListIn,thisEdge];
           nearNode.neighborsOut=[nearNode.neighborsOut,thisEdge];
           newNode.neighborsIn=[newNode.neighborsIn,thisEdge];
        
        
        else
            continue;
        end
        if (nearNode.rrtLMC > newNode.rrtLMC + thisEdge.dist &&...
        newNode.rrtParentEdge.endNode ~= nearNode && ...
        newNode.rrtLMC + thisEdge.dist < moveGoal.rrtLMC)
    
        makeParentOf(newNode, nearNode, thisEdge);
        oldLmc = nearNode.rrtLMC;
        nearNode.rrtLMC = newNode.rrtLMC + thisEdge.dist;
        
        
            if oldLmc - nearNode.rrtLMC > Q.changeThresh&&nearNode~=S.tree(1,:)
            verifyInQueue(Q, nearNode);
            end
      
        end
    end
    nodeList=[];
    addToHeap(Q.Q, newNode);
end
%for propogating changes through the graph
function rewire(Q,thisNode,hyberBallRad,changeThresh)
    deltaCost = thisNode.rrtTreeCost - thisNode.rrtLMC;
    if deltaCost <= changeThresh
        return;
    end
    cullCurrentNeighbors(thisNode, hyberBallRad);
    thisNodeInInitialNeighbors= thisNode.InitialNeighborListIn;
         thisNodeInNeighbors=thisNode.neighborsIn;
         tempNeighbors=[thisNodeInInitialNeighbors,thisNodeInNeighbors];
         m=size(tempNeighbors,2);
    for i=1:m
        neighborEdge=tempNeighbors(i);
        neighborNode=neighborEdge.startNode;
        if thisNode.rrtParentUsed && thisNode.rrtParentEdge.endNode == neighborNode
          continue;
        end
        neighborEdge=tempNeighbors(i);
        deltaCostNeighbor = neighborNode.rrtLMC - (thisNode.rrtLMC + neighborEdge.dist);
        
        if deltaCostNeighbor > 0
            neighborNode.rrtLMC = thisNode.rrtLMC + neighborEdge.dist;

            if(~neighborNode.rrtParentUsed || neighborNode.rrtParentEdge.endNode ~= thisNode)
                makeParentOf(thisNode, neighborNode, neighborEdge);
            end
            
            if neighborNode.rrtTreeCost - neighborNode.rrtLMC > changeThresh
                verifyInQueue(Q, neighborNode);
            end
        end
    end
end
%removes members of the current neighbor list of thisNode that are too far away
function cullCurrentNeighbors(thisNode,hyberBallRad)
    t=[];
    for i=1:size(thisNode.neighborsOut,2)
        if(thisNode.neighborsOut(i).dist>hyberBallRad)
            neighorNode=thisNode.neighborsOut(i).endNode;
            m=size(neighorNode.neighborsIn,2);
            for j=1:m
                if neighorNode.neighborsIn(j).startNode==thisNode
                    neighorNode.neighborsIn(j)=[];
                    break;
                end
            end
            t=[t,i];
        end
    end
    thisNode.neighborsOut(t)=[];
end
function [newNode,nodeList]=findBestParent(S, newNode, nodeList, closestNode)
      if size(nodeList,1)== 0
            if S.goalNode ~= newNode
            nodeList=closestNode;
            end
      end
      
      newNode.rrtLMC = Inf;
      newNode.rrtTreeCost = Inf;
      newNode.rrtParentUsed = false;
      
      m=size(nodeList,1);
      for i=1:m
          nearNode=nodeList(i);
          thisEdge=Edge(newNode,nearNode);
          nearNode.tempEdge=thisEdge;
        if explicitEdge2Check(S, thisEdge)
            nearNode.tempEdge.dist = Inf;
            continue;
        end
          if newNode.rrtLMC > nearNode.rrtLMC + thisEdge.dist
                % found a potentially better parrent
                newNode.rrtLMC = nearNode.rrtLMC + thisEdge.dist;
                newNode.rrtParentEdge = thisEdge;
                newNode.rrtParentUsed = true;
          end
      end
end
function makeParentOf(newParent,node,edge)

if node.rrtParentUsed
    m=size(node.rrtParentEdge.endNode.SuccessorList,2);
        for i=1:m
            if node.rrtParentEdge.endNode.SuccessorList(i)==node
                node.rrtParentEdge.endNode.SuccessorList(i)=[];
                break;
            end
        end
        
end

  node.rrtParentEdge = edge;
  node.rrtParentUsed = true;
  
  backEdge = Edge(newParent, node);
  backEdge.dist = Inf;
  newParent.SuccessorList=[newParent.SuccessorList backEdge];
end
function verifyInQueue(Q,node)
    if node.inPriorityQueue
        updateHeap(Q.Q,node);
    else
        addToHeap(Q.Q,node);
    end
end
function verifyInOSQueue(Q,node)
    if node.inPriorityQueue
        updateHeap(Q.Q,node);
        removeFromHeap(Q.Q,node);
    end
    
    if ~node.inOSQueue
        node.inOSQueue=true;
        Q.OS=[Q.OS,node];
    end
end


% propogate orphan status to all nodes in the basin(s) of attraction of 
% the nodes in Q.OS (that have higher cost). This also takes the robot
% to remember if node the robot was moving at is one of the nodes that
% has become an orphan 
 function propogateDescendants(Q,R)
    m=size(Q.OS,2);
    if m <=0
        return; 
    end
    
    n=1;
    while n<=m%add all of this' node's successors to OS Stack
        thisNode=Q.OS(n);
        SuccessorList_item=thisNode.SuccessorList;
        
        for i=1:size(SuccessorList_item,2)
            successorNode=SuccessorList_item(i).endNode;
            verifyInOSQueue(Q,successorNode);
            
        end
         m=size(Q.OS,2);
        n=n+1;
    end
    
    m=size(Q.OS,2);
    while m>=1
  %put all -out neighbors- of the nodes in OS (not including nodes
  %in OS) into Q and tell them to force rewire
        thisNode =Q.OS(m);
         thisNodeOutInitialNeighbors= thisNode.InitialNeighborListOut;
         thisNodeOutNeighbors=thisNode.neighborsOut;
         tempNeighbors=[thisNodeOutInitialNeighbors,thisNodeOutNeighbors];
       
        for i=1:size(tempNeighbors,2)
            neighborNode=tempNeighbors(i).endNode;
            if neighborNode.inOSQueue
                continue;
            end
            
            neighborNode.rrtTreeCost = Inf;
            verifyInQueue(Q, neighborNode);
        end
        if thisNode.rrtParentUsed && ~thisNode.rrtParentEdge.endNode.inOSQueue
            %add parent to the Q, unless it is in OS
            thisNode.rrtParentEdge.endNode.rrtTreeCost = Inf;
            verifyInQueue(Q, thisNode.rrtParentEdge.endNode);
        end
            m=m-1;
    end
    
    m=size(Q.OS,2);
    while m>0
% remove all nodes from OS, unmark them, and remove
% their connections to their parents, and if one was the robot's target
% then take approperiate measures
    thisNode=Q.OS(m);
    Q.OS(m)=[];
    m=m-1;
    thisNode.inOSQueue=false;
        if thisNode==R.nextMoveTarget
            R.currentMoveInvalid = true;
        end
        if thisNode.rrtParentUsed
            s=size(thisNode.rrtParentEdge.endNode.SuccessorList,2); 
            for j=1:s
                if thisNode.rrtParentEdge.endNode.SuccessorList(j)==thisNode
                    thisNode.rrtParentEdge.endNode.SuccessorList(j)=[];
                    break;
                end
            end
            thisNode.rrtParentEdge =Edge(thisNode, thisNode) ; 
            thisNode.rrtParentEdge.dist = Inf;   
            thisNode.rrtParentUsed = false;
            
        end
      thisNode.rrtTreeCost = Inf;
      thisNode.rrtLMC = Inf;
    end
 end
 
 
 %propogates cost information through the graph
function reduceInconsistency(Q,goalNode,root,hyberBallRad)
    while Q.Q.indexOfLast >0 &&(lessQ(topHeap(Q.Q),goalNode)||goalNode.rrtLMC==Inf||goalNode.rrtTreeCost==Inf||goalNode.inPriorityQueue)
        thisNode=popHeap(Q.Q);
        if thisNode.rrtTreeCost - thisNode.rrtLMC > Q.changeThresh
            recalculateLMCMineVTwo(thisNode, root, hyberBallRad);
            rewire(Q, thisNode,hyberBallRad, Q.changeThresh);
        end
    thisNode.rrtTreeCost = thisNode.rrtLMC;

    end
end
%%%%%%%%%%%%%%%%%%end of RRTX main operations%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%structure for caculating%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bubbleUp(H,n)
    if(n==1)
        return;
    end
    if n>size(H.heapNode,2)
    error('bubble failed');
    end
    parent=floor(n/2);
    while n~=1&&greaterQ(H.heapNode(parent),H.heapNode(n));
        tempNode=H.heapNode(parent);
        H.heapNode(parent)=H.heapNode(n);
        H.heapNode(n)=tempNode;
        H.heapNode(parent).priorityQueueIndex=parent;
         H.heapNode(n).priorityQueueIndex=n;
         n=parent;
         parent=floor(n/2);
    end
end
function bubbleDown(H,n)
    
  if 2*n == H.indexOfLast
    child = 2*n;
  elseif 2*n+1 > H.indexOfLast
        return;
  elseif lessQ(H.heapNode(2*n), H.heapNode(2*n+1))
        child = 2*n;
  else
        child = 2*n+1;
  end
  
  while n <= H.parentOfLast && lessQ(H.heapNode(child), H.heapNode(n))
        tempNode = H.heapNode(child);
        H.heapNode(child) = H.heapNode(n);
        H.heapNode(n)= tempNode;
        H.heapNode(child).priorityQueueIndex=child;
        H.heapNode(n).priorityQueueIndex=n;
        
        n = child;

        if 2*n == H.indexOfLast
            child = 2*n;
        elseif 2*n+1 > H.indexOfLast
            return;
        elseif lessQ(H.heapNode(2*n), H.heapNode(2*n+1))
            child = 2*n;
            else
                child = 2*n+1;
        end
  end
end
function updateHeap(H,thisNode)
    if ~thisNode.inPriorityQueue
        error('trying to update a node that is not in the heap\n');
    end
    bubbleUp(H,thisNode.priorityQueueIndex);
    bubbleDown(H,thisNode.priorityQueueIndex);
end
function addToHeap(H,thisNode)
    
   if thisNode.inPriorityQueue==false
        H.indexOfLast=H.indexOfLast+1;
        H.parentOfLast=floor(H.indexOfLast/2);
        H.heapNode=[H.heapNode,thisNode];
        thisNode.priorityQueueIndex=H.indexOfLast;
        bubbleUp(H,H.indexOfLast);
        thisNode.inPriorityQueue=true;
    end
end
function removeFromHeap(H,thisNode)
    n=thisNode.priorityQueueIndex;
    movedNode=H.heapNode(H.indexOfLast);
    H.heapNode(n)=movedNode;
    movedNode.priorityQueueIndex=n;
    H.heapNode(H.indexOfLast)=[];
    H.indexOfLast=H.indexOfLast-1;
    H.parentOfLast=floor(H.indexOfLast/2);
    if n==H.indexOfLast+1
        thisNode.inPriorityQueue=false;
        thisNode.priorityQueueIndex=-1;
        return;
    end
    bubbleUp(H,n);
    bubbleDown(H,movedNode.priorityQueueIndex)
    thisNode.inPriorityQueue=false;
    thisNode.priorityQueueIndex=-1;
end
function ret=topHeap(H)
    if H.indexOfLast<1
        ret= false ;
    else ret= H.heapNode(1);
    end
end
function ret=popHeap(H)
    if H.indexOfLast <1
        ret=false;
        return;
    end
    oldTopNode=H.heapNode(1);
    H.heapNode(1)=H.heapNode(H.indexOfLast);
    H.heapNode(1).priorityQueueIndex=1;
    H.heapNode(H.indexOfLast)=[];
    H.indexOfLast=H.indexOfLast-1;
    H.parentOfLast=floor(H.indexOfLast/2);
    bubbleDown(H,1);
    oldTopNode.inPriorityQueue=false;
    oldTopNode.priorityQueueIndex=-1;
    ret = oldTopNode;
end
function [a,b]=keyQ(node)
    g_min=min(node.rrtTreeCost, node.rrtLMC);
    a=g_min+0.0;
    b=g_min;
end
function ret=greaterQ(a,b)
   ret=false;
   [af, as]=keyQ(a);
    [bf, bs]=keyQ(b);
    if (af>bf)||(af==bf&&as>bs)||(af==bf&&as==bs&&b.isMoveGoal)
        ret=true;
    end
    
end
function ret=lessQ(a,b)
    [af, as]=keyQ(a);
    [bf, bs]=keyQ(b);
    if (af<bf)||(af==bf&&as<bs)||(af==bf&&as==bs&&a.isMoveGoal)
        ret=true;
        return;
    end
    ret=false;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%end of structure%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%detect the obstacle crash%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function ret=explicitEdge2Check(S,e)
    ret=false;
   
    m=size(S.ob,2);
    for i=1:m
    if explicitEdge3Check(S.ob(i),e,S.robotradius)
        ret=true;
        return;
    end
    end
    
end
function ret=explicitEdge3Check(ob,e,radius)
        ret=false;    
        if ob.obstacleUnused || ob.lifespan <0
        return;
        end
        distSqrd = distanceSqrdPointToSegment(ob.position,e.startNode.position,e.endNode.position);
        if distSqrd > (radius + ob.radius)^2
          return;
        end
        
        P=size(ob.polygon,1);
        if P<2
            return;
        end
        A=ob.polygon(P,1:2);
        for i=1:P
            B=ob.polygon(i,1:2);
            if segmentDistSqrd(e.startNode.position(1:2), e.endNode.position(1:2), A, B) < radius^2
                ret=true;
                return;
            end
            A=B;
        end
end
function ret=segmentDistSqrd(PA,PB,QA,QB)
      possibleIntersect = true;
      %first check if P is close to vertical
      if abs(PB(1)-PA(1))<.000001
            if (QA(1) >= PA(1) && QB(1) >= PA(1)) ...
                    || (QA(1) <= PA(1) && QB(1) <= PA(1))
                %Q is on one side of P
                possibleIntersect = false;
            end
      else
        % P is not close to vertical
        m = (PB(2) - PA(2))/(PB(1) - PA(1));

        diffA = (m*(QA(1) - PA(1)) + PA(2)) - QA(2);
        diffB = (m*(QB(1) - PA(1)) + PA(2)) - QB(2);
            if (diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0)
            % Q is either fully above or below the line containing P
            possibleIntersect = false;
            end
       end

  if possibleIntersect
    %first check if Q is close to vertical
    if abs(QB(1) - QA(1)) < .000001
      if (PA(1) >= QA(1) && PB(1) >= QA(1)) || (PA(1) <= QA(1) && PB(1) <= QA(1))
        %P is on one side of Q
        possibleIntersect = false;
      end
    else
      %Q is not close to vertical
      m = (QB(2) - QA(2))/(QB(1) - QA(1));
      diffA = (m*(PA(1) - QA(1)) + QA(2)) - PA(2);
      diffB = (m*(PB(1) - QA(1)) + QA(2)) - PB(2);
      if (diffA > 0.0 && diffB > 0.0) || (diffA < 0.0 && diffB < 0.0)
       %P is either fully above or below the line containing Q
        possibleIntersect = false;
      end
    end
  end

  if possibleIntersect
    %then there is an intersection for sure
    ret=0.0;
    return;
  end

  % when the lines do not intersect,
  %in 2D the min distance must be between one segment's end point
  %and the other segment (assuming lines are not parallel)
  ret= min([distanceSqrdPointToSegment(PA, QA, QB),...
                  distanceSqrdPointToSegment(PB, QA, QB),...
                  distanceSqrdPointToSegment(QA, PA, PB),...
                  distanceSqrdPointToSegment(QB, PA, PB)]);
end
%returns the min distance squared between the point and the segment
function ret=distanceSqrdPointToSegment(p,sp,ep)
 vx = p(1)-sp(1);
 vy = p(2)-sp(2);
 ux = ep(1)-sp(1);
 uy = ep(2)-sp(2);
 determinate =vx*ux+vy*uy;
 
 if determinate<=0
     ret = vx*vx + vy*vy;
     return ;
 else
    len = ux*ux + uy*uy;
    if determinate >= len
        ret=sum(((ep-p).^2),2);
        return;
    else
        ret=(ux*vy-uy*vx)^2 / len;
        return;
    end
 end
end
function explicitlyUnSafe=explicitNodeCheck(S,position)
    explicitlyUnSafe=false;
  
    if quickCheck(S,position)
        explicitlyUnSafe=true;
        return;
    end
    
    retCert=Inf;
    n=size(S.ob,1);
    for i=1:n
        [thisRetVal,thisCert]=explicitPointCheck2D(S.ob(i),position,retCert,S.robotradius);
        if thisRetVal
            explicitlyUnSafe=true;
        end
        if thisCert<retCert
            retCert=thisCert;
        end
    end 
end
function [unsafe, mindist]=explicitPointCheck2D(Obstacle,point,mindist,robotradius)
    unsafe=false;
    if Obstacle.obstacleUnused || Obstacle.lifespan <= 0
    return ;
    end
    
    thisdist =wdist(Obstacle.position,point)-robotradius;
    if thisdist-Obstacle.radius>mindist
        return;
    end
    if pointInPolygon(Obstacle,point)
        unsafe=true;
        mindist=0.0;
        return;
    end
    thisdist=sqrt(distToPolygonSqrd(point, Obstacle.polygon)) - robotradius;
    if thisdist<0.0
        unsafe=true;
        mindist=0.0;
        return;
    end
mindist = min(mindist,thisdist);
return;
end
%returns the distance of closest point on the bouandary of polygon to point
function mindistSqrd=distToPolygonSqrd(point,polygon)
 mindistSqrd = inf;
 p = size(polygon,1);
 startPoint = polygon(p,1:2);
 for i= 1:p
     endPoint =polygon(p,1:2);
     thisdistSqrd=distanceSqrdPointToSegment(point, startPoint, endPoint);
    if thisdistSqrd<mindistSqrd
        mindistSqrd=thisdistSqrd;
    end
    startPoint=endPoint;
 end
end
%returns the min distance squared between the point and the segment
function unsafe=quickCheck(S,point)
    unsafe=false;
    n=size(S.ob,2);
    for i=1:n
        if quickCheck2D(S.ob(i),point)
            unsafe=true;
            return;
        end
    end
end
function NodeinsideObstacle=quickCheck2D(Obstacle,point)
    NodeinsideObstacle =false;
    
    if Obstacle.obstacleUnused || Obstacle.lifespan <= 0
    return 
    end
    
    if wdist(Obstacle.position,point)>Obstacle.radius
    return
    end
    
    if pointInPolygon(Obstacle,point)
        NodeinsideObstacle=true;
        return;
    end
end
function IspointInPolygon=pointInPolygon(Obstacle,point)
    IspointInPolygon=false;
    p=size(Obstacle.polygon,1);
    if p<2
        return 
    end
    
    numCrossings =0;
    
    startPoint = Obstacle.polygon(p,1:2);
  for i = 1:p
    endPoint =Obstacle.polygon(i,1:2);
    
    %check if this edge crosses the y value of point
    if (startPoint(2) > point(2) && endPoint(2) < point(2))...
            ||...
       (startPoint(2) < point(2) && endPoint(2)> point(2))
       % it does, so now we need to see if the ray from point -> [inf, 0]
       % intersects the line.
       if startPoint(1) > point(1) && endPoint(1) > point(1)
           % definatly yes if both x coordiantes are right of the point
           numCrossings =numCrossings + 1;
      
        elseif startPoint(1) < point(1) && endPoint(1) < point(1) 
         % definatly no if both x coordiantes are left of the point 
        else
         % otherwise we need to do the "expensive" calculation
         T = 2*max(startPoint(1), endPoint(1));

         x =(-( (startPoint(1)*endPoint(2)-startPoint(2)*endPoint(1))*...
             (point(1)-T))+((startPoint(1)-endPoint(1))*(point(1)*point(2)-point(2)*T)))...
             /((startPoint(2)-endPoint(2))*(point(1)-T));

         if x > point(1)
           numCrossings = numCrossings + 1;
         end
       end
    end
    
       startPoint=endPoint;
  end
  if mod(numCrossings,2)==0
        return ;
  end
    IspointInPolygon=true;

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%end of detecting%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function d=wdist(x,y)
        
        d=norm(x-y);

end