function [ppoint, opttour1, clustermst1, totcost] = neighborbydistance(M,X,Y,CR)

% Here M is the total number of sensors
% X and Y is the co-ordinates of sensors
% CR is communication range
% implementation of singlehop tour length of a disconnected network using single M-collector

figure(110);
% rectangle('Position', [0,0,300,300])
hold on;

% plotting the position of sensor nodes

for i=1:M
    plot(X(i),Y(i),'o','MarkerSize',10);
    text(X(i),Y(i), num2str(i),'fontsize',10);
end

neghbourset = zeros(M,M);                   % Neighbor set

for i = 1:M
    for j = 1:M
        distance = sqrt((X(i) - X(j))^2 + (Y(i) - Y(j))^2);
        if distance <= CR
            neghbourset(i,j) = 1;
        end
    end
end

% Tarjan algorithm is used to find components.
% The number of components found is returned in n, and
% C is a vector indicating that a sensor node belongs to which component.

G = sparse(neghbourset);
[n, C] = graphconncomp(G);

% Count the number of points in each cluster

nofpointsineachcluster = zeros(1,n);

for i=1:n
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

clustermat = zeros(n,n);
nodeinfomat = zeros((n*(n-1))/2,3);
cluinfomat = zeros((n*(n-1))/2,3);
temparray = zeros(1,M); % Storing the information of participated node of each cluster

for i = 1 : n-1
    
    for j =  i+1 : n
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
            temparray(u) = firstcluster;
            temparray(v) = secondcluster;
            
            nodeinfomat(t,1) = u;
            nodeinfomat(t,2) = v;
            nodeinfomat(t,3) = min;
            
        end
        clustermat(i,j)= min;
        clustermat(j,i)=min;
        
        cluinfomat(t,1) = i;
        cluinfomat(t,2) = j;
        cluinfomat(t,3) = min;
        t = t + 1;
    end
end

% Count the number of points covered in each cluster

participatednode = zeros(1,n); % Contaning information about total number of node participated from each clister

for i=1:n
    ctr =0;
    for j=1:M
        if( temparray(j) == i)
            ctr = ctr + 1;
        end
    end
    participatednode(i) = ctr;
end

%minimum spanning tree for cluster

G = sparse(clustermat);
[clustermst1]=graphminspantree(G);
minst = full(clustermst1);

mst = zeros(n,n);
for i = 1:n
    for j = 1:n
        if minst(i,j) ~= 0
            mst(i,j) = minst(i,j);
            mst(j,i) = minst(i,j);
        end
    end
end

% Count number of nodes participated from each cluster after performing MST

Pnodeminmst = zeros(1,M);
for i=1:n
    for j = 1:n
        for k = 1: n*(n-1)/2
            if minst(i,j) == nodeinfomat(k,3)  && minst(i,j)~= 0  && cluinfomat(k,1) == j && cluinfomat(k,2) == i
                x1 = nodeinfomat(k,2);
                y1 = nodeinfomat(k,1);
                
                Pnodeminmst(x1)= cluinfomat(k,2);
                Pnodeminmst(y1)= cluinfomat(k,1);
            end
        end
    end
end

Pnodeinmst = zeros(1,n);
for i=1:n
    ctr =0;
    for j=1:M
        if(Pnodeminmst(j) == i)
            ctr = ctr + 1;
        end
    end
    Pnodeinmst(i) = ctr;
end

% Find the position of sensor involve in cluster graph after performing mst

% % % point1 = [];
% % % point2 = [];
% % % XC = [];
% % % YC = [];
% % % for i = 1:n
% % %     if Pnodeinmst(i) > 1 && Pnodeinmst(i) <= participatednode(i)
% % %         for j=1:M
% % %             if Pnodeminmst(j)== i
% % %                 x = X(j);
% % %                 y = Y(j);
% % %                 point1 = [point1 , x];
% % %                 point2 = [point2 , y];
% % %             end
% % %         end
% % %         
% % %         if numel(point1)<= 3
% % %             [c , r] = minboundcircle(point1,point2);
% % %         else
% % %             [c , r] = minboundcircle(point1,point2,false);
% % %         end
% % %         XC = [XC,c(1)];
% % %         YC = [YC,c(2)];
% % %         circle(c(1),c(2),r);
% % %         point1 = [];
% % %         point2 = [];
% % %     end
% % % end

% Find the nearest point from center in each cluster

% % % di1 = zeros(1,M);
% % % minIndx=1;
% % % pointpos = [];
% % % for i = 1 :numel(XC)
% % %     for j = 1:M
% % %         di1(j) = sqrt((XC(i)-X(j))^2 + (YC(i)-Y(j))^2);
% % %         if di1(j) <= di1(minIndx)
% % %             minIndx = j;
% % %             ppos = minIndx;
% % %         end
% % %     end
% % %     pointpos = [pointpos,ppos];
% % % end
% % % 
% % % % Complete cluster matrix for performing dfs
% % % 
% % % dfsclumatrix = zeros(n,n);
% % % 
% % % for i = 1:n
% % %     for j = 1:n
% % %         for k = 1:n*(n-1)/2
% % %             if Pnodeinmst(i) > 1 && Pnodeinmst(i) <= participatednode(i)
% % %                 if mst(i,j) == nodeinfomat(k,3) && mst(j,i) ~=0  && cluinfomat(k,1) == j && cluinfomat(k,2) == i
% % %                     xpos = nodeinfomat(k,1);
% % %                     y1 = nodeinfomat(k,1);
% % %                     
% % %                     for a = 1:numel(pointpos)
% % %                         val = sqrt((X(pointpos(a))-X(xpos))^2 + (Y(pointpos(a))-Y(y1))^2);
% % %                         dfsclumatrix(j,i) = val;
% % %                         dfsclumatrix(i,j) = val;
% % %                     end
% % %                 end
% % %                 if mst(i,j) == nodeinfomat(k,3) && mst(i,j) ~=0  && cluinfomat(k,1) == i && cluinfomat(k,2) == j
% % %                     xpos = nodeinfomat(k,2);
% % %                     y1 = nodeinfomat(k,2);
% % %                     
% % %                     for a = 1:numel(pointpos)
% % %                         val = sqrt((X(pointpos(a))-X(xpos))^2 + (Y(pointpos(a))-Y(y1))^2);
% % %                         dfsclumatrix(j,i) = val;
% % %                         dfsclumatrix(i,j) = val;
% % %                     end
% % %                 end
% % %             end
% % %         end
% % %         if dfsclumatrix(i,j)==0
% % %             dfsclumatrix(i,j)=mst(i,j);
% % %         end
% % %     end
% % % end

% G1 = sparse(dfsclumatrix);
G1 = sparse(mst);
d= zeros(n,n);

for i = 1:n
[order] = graphtraverse(G1, i,'Method','DFS');
for j = 1 : n
    d(i,j) = order(j);
end    
end

tlen1 = zeros(1,n);

for i = 1:n
    tl = 0;
    for j = 1:n
        min = 0;
        for k = 1 : M
            for l = 1 : M
                if j == n
                   if C(k) == d(i,j) && C(l) == d(i,1) && d(i,j) ~= d(i,1)
                    
                    if min == 0
                        min = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                    end
                    
                    temp = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                    if temp <= min
                        min = temp;
                    end
                   end
                else
                if C(k) == d(i,j) && C(l) == d(i,j+1) && d(i,j) ~= d(i,j+1)
                   
                    if min == 0
                        min = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                    end
                    
                    temp = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                    if temp <= min
                        min = temp;
                    end
                end
                end
            end
        end
        tl = tl + min;
    end
    tlen1(i)= int32(tl);
end

minid = 1;

for i= 2:n
    if tlen1(i) <= tlen1(minid)
        minid=i;
    end
end
for i = 1:n
    opttour1(i) = d(minid,i);
end

% Shortest path matrix between components for whole network

s = 1;
shortestpathmatrix = zeros(n,n);
nodeinfomat1 = zeros(n,3);
cluinfomat1 = zeros(n,3);
temparray1 = zeros(1,M); % Storing the information of participated node of each cluster

for i = 1:n-1
    for j = i+1:n
        min =0;
        if i < j && j == i+1
            
            for k = 1 : M
                for l = 1 : M
                    if C(k) == opttour1(i) && C(l) == opttour1(j) && opttour1(i) ~= opttour1(j)
                        
                        if min == 0
                            min = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                        end
                        
                        temp = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                        if temp <= min
                            min = temp;
                            u = k;
                            firstcluster = opttour1(i);
                            secondcluster = opttour1(j);
                            v = l;
                        end
                        
                    end
                end
            end
            line([X(u) X(v)], [Y(u) Y(v)], 'LineStyle', '--');
            temparray1(u) = firstcluster;
            temparray1(v) = secondcluster;
            
            nodeinfomat1(s,1) = u;
            nodeinfomat1(s,2) = v;
            nodeinfomat1(s,3) = min;
            
            cluinfomat1(s,1) = i;
            cluinfomat1(s,2) = j;
            cluinfomat1(s,3) = min;
            s = s + 1;
        end
        shortestpathmatrix(i,j)= min;
        shortestpathmatrix(j,i)=min;
    end
end

for i = n
    for j = 1
        min =0;
        for k = 1 : M
            for l = 1 : M
                if C(k) == opttour1(i) && C(l) == opttour1(j) && opttour1(i) ~= opttour1(j)
                    
                    if min == 0
                        min = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                    end
                    
                    temp = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                    if temp <= min
                        min = temp;
                        u = k;
                        firstcluster = opttour1(i);
                        secondcluster = opttour1(j);
                        v = l;
                    end
                    
                end
            end
        end
        line([X(u) X(v)], [Y(u) Y(v)], 'LineStyle', '--');
        temparray1(u) = firstcluster;
        temparray1(v) = secondcluster;
        
        nodeinfomat1(s,1) = u;
        nodeinfomat1(s,2) = v;
        nodeinfomat1(s,3) = min;
        
        cluinfomat1(s,1) = i;
        cluinfomat1(s,2) = j;
        cluinfomat1(s,3) = min;
        
    end
    shortestpathmatrix(i,j)= min;
    shortestpathmatrix(j,i)=min;
end

% cost to move between component
val = 0;
for i = 1:n
    for j = 1:n
        if shortestpathmatrix(i,j)~= 0
            val = val + shortestpathmatrix(i,j);
        end
    end
end
cost = val/2;

% Count number of nodes participated from each cluster after performing shortest path between clusters

Pnodeinsp = zeros(1,n);

for i=1:n
    ctr =0;
    for j=1:M
        if( temparray1(j) == i)
            ctr = ctr + 1;
        end
    end
    Pnodeinsp(i) = ctr;
end

% Find the position of sensor involve as entry and exit point in each component

nodepoints = [];
ppoint = [];
Nmsg = [];
point1 = [];
point2 = [];
pxy = [];
clusteroutput = [];
Tourlength = 0;

for i = 1:n
    if Pnodeinsp(i) == 1
        for l= 1:M
            if temparray1(l) == i
                ppoint = [ppoint,l];
                Nmsg = [Nmsg,nofpointsineachcluster(i)];
            end
        end
    end
if Pnodeinsp(i) > 1 && Pnodeinsp(i) <= participatednode(i)
        
for j=1:M
    if temparray1(j)== i
        x = X(j);
        y = Y(j);
        point1 = [point1 , x];
        point2 = [point2 , y];
        nodepoints = [nodepoints,j];
    end
end
         
entrypointX = point1(1);
entrypointY = point2(1);

exitpointX = point1(2);
exitpointY = point2(2);
kluster = i;
        
% % % % %

    dentry = zeros(1,nofpointsineachcluster(i)); % Distance from mobile collectors to othere polling points
    dexit = zeros(1,nofpointsineachcluster(i)); % Distance from mobile collectors to othere polling points
    ns = zeros(nofpointsineachcluster(i),nofpointsineachcluster(i));   % Neighbor set
    nns = zeros(1,nofpointsineachcluster(i));  % Number of neighbors
    costOfPollingPoints= zeros(1,nofpointsineachcluster(i)); % Cost of polling points from the mobile collector
    Coveredclusterpoint=zeros(1,nofpointsineachcluster(i)); % sensors which are covered by the mobile collectors
    CoveredclusterPollingPoints= zeros(1,nofpointsineachcluster(i)); % polling points which are traversed by the mobile collectors

% % % % %
figure(10);
hold on;
for k = 1: M
    if C(k) == kluster
        pxy = [pxy,k];
    end
end

if numel(pxy) > 2
            
for l = 1:nofpointsineachcluster(i)
    for m = 1:nofpointsineachcluster(i)
        distance = sqrt((X(pxy(l)) - X(pxy(m)))^2 + (Y(pxy(l)) - Y(pxy(m)))^2);
        if distance <= CR
            ns(l,m) = 1;
            nns(l)=nns(l)+1;
        end
    end
end

% Nodes covered by the entry and exit point

for m = 1:nofpointsineachcluster(i)
    for l=1: numel(nodepoints)
        if pxy(m) == nodepoints(l)
            for k=1:nofpointsineachcluster(i)
                if ns(m,k) == 1
                    Coveredclusterpoint(k)=1;
                end
            end
        end
    end
end

% count of covered nodes
coverednodes = 0;
 for z = 1:nofpointsineachcluster(i)
     if Coveredclusterpoint(z)==1
         coverednodes = coverednodes+1;
     end
 end

% polling points
 
ppoint = [ppoint,nodepoints];

for k = 1:numel(pxy)
for l = 1:2
    if pxy(k) == nodepoints(l)
        Nmsg = [Nmsg, nns(k)];
    end
end
end

nUncovered = nofpointsineachcluster(i)-coverednodes;
curentrypointX=entrypointX;
curentrypointY=entrypointY;

while nUncovered > 0
    
% Find the distance to the remaining polling points from entry point
    
for k=1:nofpointsineachcluster(i)
    if Coveredclusterpoint(k)==1
        dentry(k)=inf;
        dexit(k)=inf;
        nns(k) = inf;
        costOfPollingPoints(k) = inf;
    else
        dentry(k) = sqrt(   (curentrypointX - X(pxy(k)))^2 + (curentrypointY - Y(pxy(k)))^2   );
    end
end
    
% Find the distance to the remaining polling points from exit point
    
for k=1:nofpointsineachcluster(i)
    if Coveredclusterpoint(k)==1
        dentry(k)=inf;
        dexit(k)=inf;
        nns(k) = inf;
        costOfPollingPoints(k) = inf;
    else
        dexit(k) = sqrt(   (exitpointX - X(pxy(k)))^2 + (exitpointY - Y(pxy(k)))^2   );
    end
end

% Update neighbor set and number of neighbors of polling points
    
for k=1:nofpointsineachcluster(i)

    if CoveredclusterPollingPoints(k)==1
        continue;
    end

    for j = 1:nofpointsineachcluster(i)
        if ns(k,j)==1 && Coveredclusterpoint(j)
            ns(k,j) = 0;
            nns(k)=nns(k)-1;
        end

    end
end
    
% Finding the cost of each polling point from the mobile collector
    
for k=1:nofpointsineachcluster(i)
    if costOfPollingPoints(k)==inf
        continue;
    end
    if nUncovered == nns(k)
        costOfPollingPoints(k)=(nns(k)/(dentry(k)+dexit(k)));
    else
        costOfPollingPoints(k)= (nns(k)/(dentry(k)+dexit(k)));
        %((dentry(k)/nns(k))+(dexit(k)/(nUncovered-nns(k))));
    end
end
    
% Find the minimum cost polling points
    
maxIndx=1;
    
for k=1:nofpointsineachcluster(i)

    if costOfPollingPoints(k) >= costOfPollingPoints(maxIndx)
        maxIndx=k;
    end

end
    
    
clusteroutput=[clusteroutput, pxy(maxIndx)];
CoveredclusterPollingPoints(maxIndx)=1;
    
% sensors covered by the polling point
    
for k=1:nofpointsineachcluster(i)
    if ns(maxIndx,k)==1
        Coveredclusterpoint(k)=1;
    end
end
  
ppoint = [ppoint,pxy(maxIndx)];
Nmsg = [Nmsg, nns(maxIndx)]; 
nUncovered= nUncovered-nns(maxIndx);
curentrypointX=X(pxy(maxIndx));
curentrypointY=Y(pxy(maxIndx));

end

clusterpolPointCnt= numel(clusteroutput); % Count the number of polling points
                    
 if clusterpolPointCnt > 0    
     
clusteroutmat = zeros(clusterpolPointCnt, clusterpolPointCnt);
                    
for k= 1:clusterpolPointCnt
    for j= 1:clusterpolPointCnt
        dist = sqrt((X(clusteroutput(k)) - X(clusteroutput(j)))^2 + (Y(clusteroutput(k)) - Y(clusteroutput(j)))^2);
        clusteroutmat(k,j) = dist;
    end
end

G = sparse(clusteroutmat);
[Tree]=graphminspantree(G);
mst1 = full(Tree);
                    
dfsmat = zeros(clusterpolPointCnt,clusterpolPointCnt);

for k= 1:clusterpolPointCnt
    for j= 1:clusterpolPointCnt
        dfsmat(k,j) = mst1(k,j);
        dfsmat(j,k) = mst1(k,j);
    end
end

d= zeros(clusterpolPointCnt,clusterpolPointCnt);
G1 = sparse(dfsmat);
for j = 1:clusterpolPointCnt
[order] = graphtraverse(G1, j,'Method','DFS');
for k = 1 : clusterpolPointCnt
    d(j,k) = order(k);
end    
end

tlen1 = zeros(1,clusterpolPointCnt);

for l = 1:clusterpolPointCnt
    tl = 0;
    for j = 1:clusterpolPointCnt
        if j == clusterpolPointCnt
        tlen = sqrt((X(clusteroutput(d(l,j))) - X(clusteroutput(d(l,1))))^2 + (Y(clusteroutput(d(l,j))) - Y(clusteroutput(d(l,1))))^2);
        tl = tl + tlen;
        else
        tlen = sqrt((X(clusteroutput(d(l,j))) - X(clusteroutput(d(l,j+1))))^2 + (Y(clusteroutput(d(l,j))) - Y(clusteroutput(d(l,j+1))))^2);
        tl = tl + tlen;
        end
    end
    tlen1(l)= int32(tl);
end

minid = 1;
for l= 2:clusterpolPointCnt
    if tlen1(l) <= tlen1(minid)
        minid=l;
    end
end
for l = 1:clusterpolPointCnt
    disc(l) = d(minid,l);
end
% [disc, pred, closed] = graphtraverse(G1, 1,'Method','DFS');

if numel(disc)==1
                    
TlWC = sqrt((X(clusteroutput(disc(k))) - entrypointX)^2 + (Y(clusteroutput(disc(k))) - entrypointY)^2);
% disp(TlWC);
Tourlength = Tourlength + TlWC;              
line([X(clusteroutput(disc(k)))  entrypointX], [Y(clusteroutput(disc(k)))  entrypointY], 'LineStyle', '-', 'color','r');

TlWC = sqrt((X(clusteroutput(disc(k))) - exitpointX)^2 + (Y(clusteroutput(disc(k))) - exitpointY)^2);
Tourlength = Tourlength + TlWC;
line([X(clusteroutput(disc(k)))  exitpointX], [Y(clusteroutput(disc(k)))  exitpointY], 'LineStyle', '-', 'color','r');

else                  
for k = 0:clusterpolPointCnt
    if k == clusterpolPointCnt
        TlWC = sqrt((exitpointX - X(clusteroutput(disc(clusterpolPointCnt))))^2 + (exitpointY - Y(clusteroutput(disc(clusterpolPointCnt))))^2);
%         disp(TlWC);
        Tourlength = Tourlength + TlWC;
        line([entrypointX X(clusteroutput(disc(1)))], [entrypointY Y(clusteroutput(disc(1)))], 'LineStyle', '-', 'color','r');
    else
        if k == 0;
            TlWC = sqrt((entrypointX - X(clusteroutput(disc(k+1))))^2 + (entrypointY - Y(clusteroutput(disc(k+1))))^2);
%             disp(TlWC);
            Tourlength = Tourlength + TlWC;
            line([exitpointX X(clusteroutput(disc(clusterpolPointCnt)))], [exitpointY Y(clusteroutput(disc(clusterpolPointCnt)))], 'LineStyle', '-', 'color','r');
        else
            TlWC = sqrt((X(clusteroutput(disc(k))) - X(clusteroutput(disc(k+1))))^2 + (Y(clusteroutput(disc(k))) - Y(clusteroutput(disc(k+1))))^2);
%             disp(TlWC);
            Tourlength = Tourlength + TlWC;
            line([X(clusteroutput(disc(k))) X(clusteroutput(disc(k+1)))], [Y(clusteroutput(disc(k))) Y(clusteroutput(disc(k+1)))], 'LineStyle', '-', 'color','r');
        end
    end
end
end 

 else
    TlWC = sqrt((exitpointX - entrypointX)^2 + (exitpointY - entrypointY)^2);
%     disp(TlWC);
    Tourlength = Tourlength + TlWC;
    line([entrypointX exitpointX], [entrypointY exitpointY], 'LineStyle', '-', 'color','r');
end
else
    ppoint = [ppoint,nodepoints];
    Nmsg = [Nmsg, 1,1];
    TlWC = sqrt((X(pxy(1)) - X(pxy(2)))^2 + (Y(pxy(1)) - Y(pxy(2)))^2);
%     disp(TlWC);
    Tourlength = Tourlength + TlWC;
    line([X(pxy(1)) X(pxy(2))], [Y(pxy(1)) Y(pxy(2))], 'LineStyle', '-', 'color','g');
end
                    
point1 = [];
point2 = [];
pxy = [];
clusteroutput = [];
nodepoints = [];
end   
end
totcost = int32(cost)+int32(Tourlength);
% disp('total number of polling points are');

% network lifetime for network

vorder = [];
for i = 1:numel(opttour1)
    for k = 1:numel(ppoint)
        
            if C(ppoint(k)) == opttour1(i)
                vorder = [vorder,ppoint(k)];
            end
    
    end
end
[ Cnwlftime50, Cnwlftime90, Cnwlftime100] = Networklifetime(M, X, Y, ppoint, Nmsg, vorder);

end

