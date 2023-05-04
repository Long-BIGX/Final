% ---------------------------------------------------------------------
% ME 450 – Spring 2023 - Final Project
% Xiangyu Long
% Final Project.m
% 
% mark: the mark number of map nodes
% start: start point
% goal: goal ponit
% weight: the weight of every edges
% crowder: the other moving cars
% carx: the x coordinate of the object
% cary: the y coordinate of the oject
% P: the matrix of motion planning
% cocol: the x coordinate of map nodes
% corow: the y coordiante of map nodes
% mx: the x coordinate of path planning
% my: the y coordiante of path planning
% ---------------------------------------------------------------------
clc
clear variables
close all

% Initialize the environment 
mark=zeros();
start=zeros();
goal=zeros();
weight=zeros();
row=4;
column=50;
nodes=row*column;
count1=0;

% Put the number into mark matrix
for i=1:row
    for j=1:column
        count1=count1+1;
        mark(i,j)=count1;
    end
end

count1=0;
count2=0;
count3=0;

% Establish the map nodes in a weighted graph. 
for i=1:column
    for j=1:row
        if i~=column
            if j==1
                
                count1=count1+1;
                start(count1)=mark(j,i);
                count2=count2+1;
                weight(count2)=2.5;
                count3=count3+1;
                goal(count1)=mark(j,i+1);
                count1=count1+1;
                start(count1)=mark(j,i);
                count2=count2+1;
                weight(count2)=0.5;
                count3=count3+1;
                goal(count1)=mark(j+1,i+1);

            elseif j~=1 && j~=row
                
                count1=count1+1;
                start(count1)=mark(j,i);
                count2=count2+1;
                weight(count2)=2.5;
                count3=count3+1;
                goal(count1)=mark(j-1,i+1);
                count1=count1+1;
                start(count1)=mark(j,i);
                count2=count2+1;
                weight(count2)=1;
                count3=count3+1;
                goal(count1)=mark(j,i+1);
                count1=count1+1;
                start(count1)=mark(j,i);
                count2=count2+1;
                weight(count2)=0.5;
                count3=count3+1;
                goal(count1)=mark(j+1,i+1);

            elseif j==row
                
                count1=count1+1;
                start(count1)=mark(j,i);
                count2=count2+1;
                weight(count2)=2.5;
                count3=count3+1;
                goal(count1)=mark(j-1,i+1);
                count1=count1+1;
                start(count1)=mark(j,i);
                count2=count2+1;
                weight(count2)=0.5;
                count3=count3+1;
                goal(count1)=mark(j,i+1);
            end
        end
    end
end

cocol=zeros();
corow=zeros();
count1=0;
count2=row+1;
carx=1;
cary=row;

% Establish the map nodes
for i=1:row
    count2=count2-1;
    for j=1:column
        count1=count1+1;
        cocol(count1)=j;
        corow(count1)=count2;
    end
end

%Produce the obstacle nodes
obs=80;
crowder=[1 carx];
k=find(crowder==carx);
c=find(crowder>=150);
% Let the obstacles not to coincide with start point
while isempty(k)==0 %|| isempty(c)==0
crowder=sort(randi([1,nodes-1],1,obs));
crowder=unique(crowder);
k=find(crowder==carx);
c=find(crowder>=150);
end

figure;
axis equal;
axis off;

% Draw each lane
hold on;
line([0, column], [4.5, 4.5],'Color','k','lineWidth',2,'lineStyle','-')
line([0, column], [3.5, 3.5],'Color','k','lineWidth',1,'lineStyle','--')
line([0, column], [2.5, 2.5],'Color','k','lineWidth',1,'lineStyle','--')
line([0, column], [1.5, 1.5],'Color','k','lineWidth',1,'lineStyle','--')
line([0, column], [0.5, 0.5],'Color','k','lineWidth',2,'lineStyle','-')

h=plot(cocol(crowder),corow(crowder),'o');
H=plot(carx,cary,'*');

flag1=start;
flag2=goal;
flag3=weight;
flag=zeros();
count1=0; 
%Note the obstacle nodes
for j=1:numel(start)
    for i=1:numel(crowder)
        if start(j)==crowder(i) || goal(j)==crowder(i)
            count1=count1+1;
            flag(count1)=j;
        end
    end
end
% Cancel them in the map nodes
flag1(flag)=[];
flag2(flag)=[];
flag3(flag)=[];

%Generate the shortest path
Goal=max(max(flag1),max(flag2));
Start=carx;
G = graph(flag1,flag2,flag3);
P = shortestpath(G,Start,Goal);

%Portray the shortest path
mx=zeros();
my=zeros();
for k=1:numel(P)
    count1=P(k);
    mx(k)=cocol(count1);
    my(k)=corow(count1);
end

L=line(mx,my,'lineWidth',1,'lineStyle','-.');
trajectory = plot(carx,cary, 'MarkerSize', 20, 'Color', 'red','lineWidth',1,'lineStyle','-');

%To show each step for every iteration
for i=1:100
    
    set(h, 'XData', cocol(crowder), 'YData', corow(crowder));
    set(L,'XData', mx, 'YData', my);
    set(H,'XData', carx, 'YData', cary); 
    set(trajectory, 'XData', [get(trajectory, 'XData'), carx], 'YData', [get(trajectory, 'YData'), cary]);
    pause(0.5);
    
    %Colission detection
    [carx,cary,crowder]=Transitionmodel(carx,cary,mx,my, cocol, corow,crowder);
    
    %Repeat the step about producing new obstacles.
    flag1=start;
    flag2=goal;
    flag3=weight;
    flag=zeros();
    count1=0; 

    for j=1:numel(start)
        for k=1:numel(crowder)
            if start(j)==crowder(k) || goal(j)==crowder(k)
                count1=count1+1;
                flag(count1)=j;
            end
        end
    end

    flag1(flag)=[];
    flag2(flag)=[];
    flag3(flag)=[];
    Goal=max(max(flag1),max(flag2));

    %Update the new start point
    if cary==4 && carx<=50
            Start=mark(1,carx);
        elseif cary==3 && carx<=50
            Start=mark(2,carx);
        elseif cary==2 && carx<=50
            Start=mark(3,carx);
        elseif cary==1 && carx<=50
            Start=mark(4,carx);
    end
    G = graph(flag1,flag2,flag3);
    if cary~=1
        P = shortestpath(G,Start,Goal);
    end  
    
    mx=zeros();
    my=zeros();
    for k=1:numel(P)
        count1=P(k);
        mx(k)=cocol(count1);
        my(k)=corow(count1);
    end
    % In case the reseach point not to exceed the map
    if carx>=50
        carx=50;
        break
    end
end

axis equal;
axis off;

%To show the final results
if carx~=50 && cary~=1
    disp('Failed!');
elseif carx==50 && cary==1
    disp('Success!');
else
    disp('Crushed!');
end


