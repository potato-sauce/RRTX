function ob=readObstacle(filename)
        a=fopen(filename,'r');
        nums=str2num(fgetl(a));
        
        for i=1:nums
            
            N=str2num(fgetl(a));
            polygon=zeros(N,2,'double');
            
            for j=1:N
                polygon(j,:)=str2array(fgetl(a));
            end
            ob(i)=Obstacle();
            ob(i).polygon=polygon;
            ob(i).BehavorType=str2num(fgetl(a));
            ob(i).startTime=0.0;
            ob(i).position=[(max(polygon(:,1)) + min(polygon(:,1)))/2.0 (max(polygon(:,2)) + min(polygon(:,2)))/2.0];
            ob(i).radius=sqrt(max(sum(bsxfun(@minus,polygon,ob(i).position).^2,2)));
            ob(i).lifespan=Inf;
            if ob(i).BehavorType == 0
                ob(i).senseableObstacle = false;
                ob(i).obstacleUnusedAfterSense = false;
                ob(i).obstacleUnused = false;
                
            elseif ob(i).BehavorType == -1
            ob(i).senseableObstacle = true;
            ob(i).obstacleUnusedAfterSense = true;
            ob(i).obstacleUnused = false;
            
            elseif ob(i).BehavorType == 1  
            ob(i).senseableObstacle = true;
            ob(i).obstacleUnusedAfterSense = false;
            ob(i).obstacleUnused = true;
            
            else
            error('unknown behavoiur type')
            end
        end
        
        fclose(a);
end

function s=str2array(Str)
     p=1;
     L=0;
     Lmax=length(Str);
     
     while p<= Lmax
         if p==Lmax || Str(p) ==',' ...
                 || strcmp(Str(p),'\n') ||strcmp(Str(p),'\0')
             L=L+1;
             if Str(p)~=','
                 break;
             end
         end
         p=p+1;
     end
     p=1;
     q=1;
     i=1;
     s=zeros(1,L,'double');
     while i<=L
         while q<Lmax&&Str(q)~=','&&~strcmp(Str(p),'\n')&&~strcmp(Str(p),'\0')
             q=q+1;
         end
         if Str(q) ==',' ||strcmp(Str(q),'\n') ||strcmp(Str(q),'\0')
             s(i)=str2double(Str(p:q-1));
             q=q+1;
             p=q;
         else
             s(i)=str2double(Str(p:q));
         end
         i=i+1;
     end

end