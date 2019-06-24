function [ optimaltour, mstofcluster, clustertourcost] = optimaltour(M,X,Y,CR)

% Here tarjan algorithm is used to find connected component.

% The number of cluster found is returned in numofcluster, and 
% C is a vector indicating to which cluster each node belongs.

neighboursets = zeros(M,M);   % Neighbor set

for i = 1:M
for j = 1:M
        distance = sqrt((X(i) - X(j))^2 + (Y(i) - Y(j))^2);
        if distance <= CR
          neighboursets(i,j) = 1;
        end
end
end
Graph = sparse(neighboursets);
[numofcluster, C] = graphconncomp(Graph);
disp(numofcluster);
% Count the number of points in each cluster

nofpointsineachcluster = zeros(1,numofcluster);

for i=1:numofcluster
    ctr =0;
    for j=1:M
        if( C(j) == i)
            ctr = ctr + 1;
        end
    end
    nofpointsineachcluster(i) = ctr;
end


% calculate the distance between each point of differentcluster

 t=1;
 
clustermatrix = zeros(numofcluster,numofcluster);

nodeinfomatrix = zeros((numofcluster*(numofcluster-1))/2,3);

cluinfomatrix = zeros((numofcluster*(numofcluster-1))/2,3); 

participatednode = zeros(1,M); % Storing the information of participated node of each cluster

for i = 1 : numofcluster-1
    
    for j =  i+1 : numofcluster
        min =0;
        if i < j
            
            for k = 1 : M
                for l = 1 : M
                    if C(k) == i && C(l) == j && i ~= j
                        
                        if min == 0
                            min = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                        end
                        
                        temp = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                        if temp <= min
                            min = temp;
                            u = k;
                            firstcluster = i;
                            secondcluster = j;
                            v = l;
                        end
                        
                    end
                end
            end
            participatednode(u) = firstcluster;
            participatednode(v) = secondcluster;
            
            nodeinfomatrix(t,1) = u;
            nodeinfomatrix(t,2) = v;
            nodeinfomatrix(t,3) = min;
           
        end
        clustermatrix(i,j)= min;
        clustermatrix(j,i)=min;
        
        cluinfomatrix(t,1) = i;
        cluinfomatrix(t,2) = j;
        cluinfomatrix(t,3) = min;
         t = t + 1;
    end
end


%minimum spanning tree for cluster

G = sparse(clustermatrix);
[mstofcluster]=graphminspantree(G);
minst = full(mstofcluster);


mst = zeros(numofcluster,numofcluster);
for i = 1:numofcluster
    for j = 1:numofcluster
        if minst(i,j) ~= 0
            mst(i,j) = minst(i,j);
            mst(j,i) = minst(i,j);
        end
    end
end

% Count number of nodes participated from each cluster after performing MST

Pnodeinmst = zeros(1,numofcluster);
for i=1:numofcluster
    ctr =0;
    for j=1:numofcluster
        if( mst(j,i) ~= 0)
            ctr = ctr + 1;
        end
    end
    Pnodeinmst(i) = ctr;
end

%% Find the position of sensor involve in cluster graph after performing mst
communicationpoint = [];
point1 = [];
point2 = [];
XC = [];
YC = [];
for i = 1:numofcluster
    if Pnodeinmst(i) > 1 && Pnodeinmst(i) <= nofpointsineachcluster(i)
        for j = 1:numofcluster
            for k = 1:numofcluster*(numofcluster-1)/2
                if mst(i,j) == nodeinfomatrix(k,3) && mst(j,i) ~=0  && cluinfomatrix(k,1) == j && cluinfomatrix(k,2) == i
                    xpos = nodeinfomatrix(k,2);
                    ypos = nodeinfomatrix(k,2);
                    x = X(xpos);
                    y = Y(ypos);
                    point1 = [point1 , x];
                    point2 = [point2 , y];
                    communicationpoint = [communicationpoint ,nodeinfomatrix(k,1); ];
                end
                if mst(i,j) == nodeinfomatrix(k,3) && mst(i,j) ~=0  && cluinfomatrix(k,1) == i && cluinfomatrix(k,2) == j
                    xpos = nodeinfomatrix(k,1);
                    ypos = nodeinfomatrix(k,1);
                    x = X(xpos);
                    y = Y(ypos);
                    point1 = [point1 , x];
                    point2 = [point2 , y];
                    communicationpoint = [communicationpoint ,nodeinfomatrix(k,2); ];
                end
                
            end
            
        end
%        [c , r] = circumcenter(point1,point2);
        if numel(point1)<= 3
            [c , r] = minboundcircle(point1,point2);
        else
            [c , r] = minboundcircle(point1,point2,false);
        end
        XC = [XC,c(1)];
        YC = [YC,c(2)];
%         t = linspace(0,2*pi,200);
%         xc = r*cos(t) + c(1);
%         yc = r*sin(t) + c(2);
%         figure(6);
%         plot(c(1),c(2),'*','MarkerSize',10)
%         plot(xc,yc,'r')
        point1 = [];
        point2 = [];
%         XC = [];
%         YC = [];

    end
end

% Find the nearest point from center in each cluster

di1 = zeros(1,M);
minIndx=1;
pointpos = [];
for i = 1 :numel(XC) 
    for j = 1:M
        di1(j) = sqrt((XC(i)-X(j))^2 + (YC(i)-Y(j))^2);
        if di1(j) <= di1(minIndx)
            minIndx = j;
            ppos = minIndx;
        end
    end
    pointpos = [pointpos,ppos];
end

dfsclumatrix = zeros(numofcluster,numofcluster);

for i = 1:numofcluster
    for j = 1:numofcluster
        for k = 1:numofcluster*(numofcluster-1)/2
            if Pnodeinmst(i) > 1 && Pnodeinmst(i) <= nofpointsineachcluster(i)
                if mst(i,j) == nodeinfomatrix(k,3) && mst(j,i) ~=0  && cluinfomatrix(k,1) == j && cluinfomatrix(k,2) == i
                    xpos = nodeinfomatrix(k,1);
                    ypos = nodeinfomatrix(k,1);
                    
                    for a = 1:numel(pointpos)
                        val = sqrt((X(pointpos(a))-X(xpos))^2 + (Y(pointpos(a))-Y(ypos))^2);
                        dfsclumatrix(j,i) = val;
                        dfsclumatrix(i,j) = val;
                    end
                end
                if mst(i,j) == nodeinfomatrix(k,3) && mst(i,j) ~=0  && cluinfomatrix(k,1) == i && cluinfomatrix(k,2) == j
                    xpos = nodeinfomatrix(k,2);
                    ypos = nodeinfomatrix(k,2);
                    
                    for a = 1:numel(pointpos)
                        val = sqrt((X(pointpos(a))-X(xpos))^2 + (Y(pointpos(a))-Y(ypos))^2);
                        dfsclumatrix(j,i) = val;
                        dfsclumatrix(i,j) = val;
                    end
                end
            end
        end
        if dfsclumatrix(i,j)==0
            dfsclumatrix(i,j)=mst(i,j);
        end
    end
end

 
G1 = sparse(dfsclumatrix);
[optimaltour] = graphtraverse(G1, 1,'Method','DFS');

tourcost = 0;
for i = 1:numofcluster-1
    value = dfsclumatrix(optimaltour(i),optimaltour(i+1));
    tourcost = tourcost + value;
end 
clustertourcost = int32(tourcost);
disp('Tour cost to visit all the clusters.');
disp(clustertourcost);

end


