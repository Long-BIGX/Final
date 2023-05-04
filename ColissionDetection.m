% ---------------------------------------------------------------------
% ME 450 – Spring 2023 - Final Project
% Xiangyu Long
% ColissionDetection.m
% 
% Input:
% x: the x coordinate of the object
% y: the y coordinate of the oject
% mx: the x coordinate of path planning
% my: the y coordiante of path planning
% c: the x coordinate of map nodes
% r: the y coordinate of map nodes
% Output:
% x: the updated x coordinate of the object
% y: the updated y coordinate of the oject
% n: the updated coordinate of obstacles
% ---------------------------------------------------------------------

function [x,y,n] = ColissionDetection (x,y,mx,my,n,P)
% The process of colission detection
if isempty(P)==1 || numel(P)<2 || P(2)-P(1)<=50 || y==1 % while there's no optimal path
    if y==4 
           x=x+4;% stay in the current lane with the matching speed in the fastest lane
           y=4;
        elseif y==3 % second lane
            x=x+3;
            y=3;
        elseif y==2 % thrid lane
            x=x+2;
            y=2;
        elseif y==1 % exit lane
            x=x+1;
            y=1;
    end
elseif isempty(P)==0 % while there's a optimal path
    a=find(n+3==P(2),1);
    b=find(n+2==P(2),1);% To detect that if there's going to have a incoming obstacle
    c=find(n+1==P(2),1);
    if y==4 && isempty(a)==1 % if it is, then fallow the otimal path
        x=mx(2);
        y=my(2);
    elseif y==4 && isempty(a)==0 % if it is not, then stay in the current lane
        x=x+4;
        y=4;
    end
    if y==3 && isempty(b)==1
        x=mx(2);
        y=my(2);
    elseif y==3 && isempty(a)==0
        x=x+3;
        y=3;
    end
    if y==2 && isempty(c)==1
        x=mx(2);
        y=my(2);
    elseif y==2 && isempty(a)==0
        x=x+2;
        y=2;
    end
end


for k=1:numel(n) % The movement of obstacles of every iteration in each lane
        if n(k)>=1 && n(k)<=46
                n(k)=n(k)+4;
        elseif n(k)==47
                n(k)=1;
        elseif n(k)==48
                n(k)=2;
        elseif n(k)==49
                n(k)=3;
        elseif n(k)==50
                n(k)=4;
        elseif n(k)>50 && n(k)<=97
            n(k)=n(k)+3;
        elseif n(k)==98
                n(k)=51;
        elseif n(k)==99
                n(k)=52;
        elseif n(k)==100
                n(k)=53;
        elseif n(k)>100 && n(k)<=148
            n(k)=n(k)+2;
        elseif n(k)==149
                n(k)=101;
        elseif n(k)==150
                n(k)=102;
        elseif n(k)>150 && n(k)<=199
            n(k)=n(k)+1;
        elseif n(k)==200
                n(k)=151;
        end
end
 
end