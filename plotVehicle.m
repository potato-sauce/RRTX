
% plots an aircraft at a pointing at b that is c big
function plotVehicle(f, a, b, c, clr, sr, clrB)

% skinney aircraft
%     x = [0 .1 .1  1  1   .1  .05  .3 .3];  
%     y = [1 .8 .3 .2 -.1 -.1 -.8 -.83 -.95];

    
  % fat aircraft  
    x = [0 .2 .2  1  1   .2  .2  .5 .5];
    y = [1 .8 .4 .4 -.1 -.1 -.7 -.7 -1];

% % satalite/rover
%     x = [0 .2 .5  .6  .6  1  1  .6 .6  .5 .2  0];
%     y = [.7 .5 .2  0   .7   .7 -.7  -.7  0 -.2 -.5 -.7];
   

    x = c*[x -fliplr(x)];
    y = c*[y fliplr(y)];
    
    
    xy = [x;y];
    
    if a ~= b
        theta = -atan2(b(1)-a(1), b(2)-a(2));
    else
        theta = 0
    end
    
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    
    
    xy_hat = R*xy;
    
      %figure(f)
    %plot(xy_hat(1,:) + a(1), xy_hat(2,:) + a(2) , clr)
    fill(xy_hat(1,:) + a(1),xy_hat(2,:) + a(2) , clr, 'EraseMode', 'none', 'EdgeColor', 'none')
    %fill3(xy_hat(1,:) + a(1),xy_hat(2,:) + a(2), ones(size(xy_hat(1,:))) , clr, 'EdgeColor', 'none')

    
    if ~isnan(sr)
        steps = 20;
        phis = 0:2*pi/steps:2*pi;
    
        xs = sr*cos(phis);
        ys = sr*sin(phis);
    
        plot(xs + a(1), ys + a(2) , 'k')  
        plot(xs + a(1), ys + a(2) , clrB)
    end
end

