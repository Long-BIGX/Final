% ---------------------------------------------------------------------
% ME 450 – Spring 2023 - Final Project
% Xiangyu Long
% Transitionmodel.m
% 
% Input:
% x: the x coordinate of the object
% y: the y coordinate of the oject
% mx: the x coordinate of path planning
% my: the y coordiante of path planning
% c: the x coordinate of map nodes
% r: the y coordinate of map nodes
% ---------------------------------------------------------------------

function [x,y,n] = Transitionmodel (x,y,mx,my,c, r,n)

if numel(my)<2 %The moving pattern of the research point when it doesn't need to change lane
        if y==4 
           x=x+4;
        elseif y==3 
            x=x+3;
        elseif y==2
            x=x+2;
        elseif y==1
            x=x+1;
        end
else
       % Colission detection in each lane
        if my(2)<y && y==4
            for k=1:numel(n) %if the next step will be coincided with other obstacles, 
                                % then keep moving on the current lane.
                if n(k)+3<=200
                    x=mx(2);
                    y=my(2);
                    if (c(n(k)+3)==x && r(n(k)+3)==y) 
                        x=x+4;

                    end
                end
            end
        elseif my(2)<y && y==3
            for k=1:numel(n)
                if n(k)+2<=200
                    x=mx(2);
                    y=my(2);
                    if c(n(k)+2)==x && r(n(k)+2)==y 
                        x=x+3;

                    end
                end
            end
        elseif my(2)<y && y==2
            for k=1:numel(n)
                if n(k)+1<=200
                    x=mx(2);
                    y=my(2);
                    if c(n(k)+1)==x && r(n(k)+1)==y 
                        x=x+2;


                    end
                end
            end
        elseif my(2)>y && y==1
            for k=1:numel(n)
                if n(k)+2<=200
                    x=mx(2);
                    y=my(2);
                    if c(n(k)+2)==x && r(n(k)+2)==y 
                        x=x+1;

                    end
                end
            end
        elseif my(2)>y && y==2
            for k=1:numel(n)
                if n(k)+3<=200
                    x=mx(2);
                    y=my(2);
                    if (c(n(k)+3)==x && r(n(k)+3)==y) 
                        x=x+2;
       

                    end
                end
            end
        elseif my(2)>y && y==3
            for k=1:numel(n)
                if n(k)+4<=200
                    x=mx(2);
                    y=my(2);
                    if (c(n(k)+4)==x && r(n(k)+4)==y) 
                        x=x+3;
  
                    end
                end
            end
        elseif my(2)==y && y==4
            x=x+4;
        elseif my(2)==y && y==3
            x=x+3;
        elseif my(2)==y && y==2
            x=x+2;
        elseif my(2)==y && y==1
            x=x+1;
        end
end

for k=1:numel(n) % The movement of obstacles in each lane
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