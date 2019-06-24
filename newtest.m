% implementation of singlehop tour length of a Component based data gathering algorithm using single M-collector
fid=fopen('output.txt','w');
fprintf(fid,'TSP Search\tPSO Length\tCuckoo Length\tFirefly Length\n\r');
fclose(fid);
for z=1:2
clear classes;
clear all;
close all;


N= 24;                   % number of sensors
CR = 75;                %communication range

%% Input through file

fid = fopen('sample2.txt','r');
X=fscanf(fid,'%d',N);
Y=fscanf(fid,'%d',N);
fclose(fid);

%% Random Input generator 

%  X = randi([0 600],N,1);
%  Y = randi([0 600],N,1);

adjMatrix=zeros(N,N);                   % Incident Matrix
neghbourset = zeros(N,N);               % Neighbor set

% figure(10);
rectangle('Position', [0,0,700,700])

hold on;

% plot all the sensor nodes

for i=1:N
    plot(X(i),Y(i),'o','MarkerSize',5);
    text(X(i),Y(i), num2str(i),'fontsize',10);
end

% Calcullating the neighbor set of each sensor nodes
count = zeros(N);
for i = 1:N
    for j = 1:N
        distance = sqrt((X(i) - X(j))^2 + (Y(i) - Y(j))^2);
        adjMatrix(i,j)=distance;
        if distance <= CR
               line([X(i) X(j)], [Y(i) Y(j)], 'LineStyle', ':');
               count(i) = count(i) +1;
               count(j) = count(i) +1;
            neghbourset(i,j) = 1;
        end
    end
end
% The number of components found is returned in n, and
% C is a vector indicating to which component each sensor node belongs.

G = sparse(neghbourset);
[n, C] = graphconncomp(G);
% Count the number of points in each cluster

nofpointsineachcluster = zeros(1,n);

for i=1:n
    ctr =0;
    for j=1:N
        if( C(j) == i)
            ctr = ctr + 1;
        end
    end
    nofpointsineachcluster(i) = ctr;
end

% calculate the distance between each point of differentcluster

t=1;
q = 1;

clustermat = zeros(n,n);

nodeinfomat = zeros((n*(n-1))/2,3);

cluinfomat = zeros((n*(n-1))/2,3);

temparray = zeros(1,N); 
 x1 = zeros(n);
 y1 = zeros(n);
%Storing the information of participated node of each cluster

for i = 1 : n-1
    
    for j =  i+1 : n
        min =0;
        if i < j
            
            for k = 1 : N
                for l = 1 : N
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
            % line([X(u) X(v)], [Y(u) Y(v)], 'LineStyle', ':');
            temparray(u) = firstcluster;
            temparray(v) = secondcluster;
            
            nodeinfomat(t,1) = u;
            nodeinfomat(t,2) = v;
            nodeinfomat(t,3) = min;
            x1(t) = X(u);
            y1(t) = Y(u);
            t = t + 1;
            x1(t) = X(v);
            X1(t) = Y(v)
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
    for j=1:N
        if( temparray(j) == i)
            ctr = ctr + 1;
        end
    end
    participatednode(i) = ctr;
end

%minimum spanning tree for cluster

G = sparse(clustermat);
[Tree, pred1]=graphminspantree(G);
minst = full(Tree);
mst = zeros(n,n);
for i = 1:n
    for j = 1:n
        if minst(i,j) ~= 0
            mst(i,j) = minst(i,j);
            mst(j,i) = minst(i,j);
        end
    end
end

% Representation of mst
for i = 1:n
    for j = 1:n
        for k =1: n*(n-1)/2
            if minst(i,j) == nodeinfomat(k,3)  && minst(i,j)~= 0  && cluinfomat(k,1) == j && cluinfomat(k,2) == i
                xpos = nodeinfomat(k,2);
                y1 = nodeinfomat(k,1);
%                 line([X(xpos) X(y1)], [Y(xpos) Y(y1)], 'LineStyle', '-', 'color','r');
            end
        end
    end
end

% Identify nodes participated from each cluster after performing MST
Pnodeminmst = zeros(1,N);
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
% Count the no. of nodes participated in each component along MST
Pnodeinmst = zeros(1,n);
for i=1:n
    ctr =0;
    for j=1:N
        if(Pnodeminmst(j) == i)
            ctr = ctr + 1;
        end
    end
    Pnodeinmst(i) = ctr;
end
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
        for k = 1 : N
            for l = 1 : N
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
discl= (n);
for i = 1:n
    disc1(i) = d(minid,i);
end
%% Shortest path matrix between components for whole network
s = 1;
shortestpathmatrix = zeros(n,n);
nodeinfomat1 = zeros(n,3);

cluinfomat1 = zeros(n,3);

temparray1 = zeros(1,N);
% Storing the information of participated node of each cluster
for i = 1:n-1
    for j = i+1:n
        min =0;
        if i < j && j == i+1
            
            for k = 1 : N
                for l = 1 : N
                    if C(k) == disc1(i) && C(l) == disc1(j) && disc1(i) ~= disc1(j)
                        
                        if min == 0
                            min = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                        end
                        
                        temp = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                        if temp <= min
                            min = temp;
                            u = k;
                            firstcluster = disc1(i);
                            secondcluster = disc1(j);
                            v = l;
                        end
                        
                    end
                end
            end
%              line([X(u) X(v)], [Y(u) Y(v)], 'LineStyle', '-');
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
 if n > 1
for i = n
    disp(i);
    for j = 1
        min =0;
        for k = 1 : N
            for l = 1 : N
                if C(k) == disc1(i) && C(l) == disc1(j) && disc1(i) ~= disc1(j)
                    
                    if min == 0
                        min = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                    end
                    
                    temp = sqrt ( ( X(k)- X(l))^2 +(Y(k)-Y(l))^2);
                    if temp <= min
                        min = temp;
                        u = k;
                        firstcluster = disc1(i);
                        secondcluster = disc1(j);
                        v = l;
                    end
                    
                end
            end
        end
%         line([X(u) X(v)], [Y(u) Y(v)], 'LineStyle', '-');
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
 end
%% cost to move between component
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
    for j=1:N
        if( temparray1(j) == i)
            ctr = ctr + 1;
        end
    end
    Pnodeinsp(i) = ctr;
end

% Find the position of sensor involve in cluster graph after performing mst
nodepoints = [];
ppoints = [];
Nmsg = [];
point1 = [];
point2 = [];
pxy = [];
pX = [];
pY = [];
x1 = [];
y1 = [];
clusteroutput = [];
Tourlength = 0;

%   if n == 1
%    %node covered by node with max neighbour
%    coverednodes = zeros(N);
%    coverdpollling = zeros(1,N);
%     nset = zeros(N,N);
%     nnset = zeros(1,N);
%     uncoverednodes = N;
%   while(uncoverednodes > 0)
%      mx = 0;
%      for i = 1:N
%        if count(i) > mx
%        mx = count(i);   
%        end    
%      end
%    for i = N
%        if nset(mx,i) == 1
%            coverednodes(i) = 1;
%        end
%    end
%    % count number of covered nodes
%    cnt = 0;
%    for i = 1:N
%        if coverednodes(i) == 1
%            cnt = cnt +1;
%        end
%    end
%    uncoverednodes = N - cnt;
%    % update neighbour set
%    for k=1:N
% 
%     for j = 1:N
%         if nset(k,j)==1 && coverdnode(j)
%             nset(k,j) = 0;
%             nnset(k)=nns(k)-1;
%         end
%        
%     end
%    end
%   end
%   end
for i = 1:n    
    if Pnodeinsp(i) == 1
      for l= 1:N
          if temparray1(l) == i
             ppoints = [ppoints,l];
             Nmsg = [Nmsg,nofpointsineachcluster(i)];
             a = X(l);
             b = Y(l);
             x1 = [x1,a];
             y1 = [y1,b];
          end
      end
    end 
    if Pnodeinsp(i) > 1 && Pnodeinsp(i) <= participatednode(i)
        
        for j=1:N
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
        x1 = [x1,entrypointX];
        y1 = [y1,entrypointY]
        x1 = [x1,exitpointX];
        y1 = [y1,exitpointY];
        kluster = i;
        
        % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
        
        dentry = zeros(1,nofpointsineachcluster(i)); % Distance from mobile collectors to othere polling points
        dexit = zeros(1,nofpointsineachcluster(i)); % Distance from mobile collectors to othere polling points
        ns = zeros(nofpointsineachcluster(i),nofpointsineachcluster(i));   % Neighbor set
        nns = zeros(1,nofpointsineachcluster(i));  % Number of neighbors
        costOfPollingPoints= zeros(1,nofpointsineachcluster(i)); % Cost of polling points from the mobile collector
        Coveredclusterpoint=zeros(1,nofpointsineachcluster(i)); % sensors which are covered by the mobile collectors
        CoveredclusterPollingPoints= zeros(1,nofpointsineachcluster(i)); % polling points which are traversed by the mobile collectors
        
        % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
        for k = 1: N
            if C(k) == kluster
                pxy = [pxy,k];
            end
        end
  
if numel(pxy) > 2
            
for l = 1:nofpointsineachcluster(i)
    for m = 1:nofpointsineachcluster(i)
        if pxy(l) == nodepoints(1) && pxy(m) == nodepoints(2) 
            continue;
        end
        if pxy(l) == nodepoints(2) && pxy(m) == nodepoints(1)
            continue;
        end
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

ppoints = [ppoints,nodepoints];
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
% x1 = [x1,curentrypointX];
% y1 = [y1,curentrypointY];
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
        costOfPollingPoints(k)=(dentry(k)/nns(k));
    else
        costOfPollingPoints(k)=((dentry(k)/nns(k))+(dexit(k)/(nUncovered-nns(k))));
    end
end
    
% Find the minimum cost polling points
    
minIndx=1;
    
for k=1:nofpointsineachcluster(i)

    if costOfPollingPoints(k) <= costOfPollingPoints(minIndx)
        minIndx=k;
    end

end
    
    
clusteroutput=[clusteroutput, pxy(minIndx)];
CoveredclusterPollingPoints(minIndx)=1;
    
% sensors covered by the polling point
    
for k=1:nofpointsineachcluster(i)
    if ns(minIndx,k)==1
        Coveredclusterpoint(k)=1;
    end
end
ppoints = [ppoints,pxy(minIndx)];
Nmsg = [Nmsg, nns(minIndx)]; 
nUncovered= nUncovered-nns(minIndx);
curentrypointX=X(pxy(minIndx));
curentrypointY=Y(pxy(minIndx));
 x1 = [x1,curentrypointX];
 y1 = [y1,curentrypointY];
end
clusterpolPointCnt= numel(clusteroutput);

 %  if clusterpolPointCnt > 0    
%      
% clusteroutmat = zeros(clusterpolPointCnt, clusterpolPointCnt);
%                     
% for k= 1:clusterpolPointCnt
%     for j= 1:clusterpolPointCnt
%         dist = sqrt((X(clusteroutput(k)) - X(clusteroutput(j)))^2 + (Y(clusteroutput(k)) - Y(clusteroutput(j)))^2);
%         clusteroutmat(k,j) = dist;
%     end
% end
% 
% G = sparse(clusteroutmat);
% [Tree]=graphminspantree(G);
% mst1 = full(Tree);
%                     
% dfsmat = zeros(clusterpolPointCnt,clusterpolPointCnt);
% 
% for k= 1:clusterpolPointCnt
%     for j= 1:clusterpolPointCnt
%         dfsmat(k,j) = mst1(k,j);
%         dfsmat(j,k) = mst1(k,j);
%     end
% end
% 
% d= zeros(clusterpolPointCnt,clusterpolPointCnt);
% G1 = sparse(dfsmat);
% for j = 1:clusterpolPointCnt
% [order] = graphtraverse(G1, j,'Method','DFS');
% for k = 1 : clusterpolPointCnt
%     d(j,k) = order(k);
% end    
% end
% 
% tlen1 = zeros(1,clusterpolPointCnt);
% 
% for l = 1:clusterpolPointCnt
%     tl = 0;
%     for j = 1:clusterpolPointCnt
%         if j == clusterpolPointCnt
%         tlen = sqrt((X(clusteroutput(d(l,j))) - X(clusteroutput(d(l,1))))^2 + (Y(clusteroutput(d(l,j))) - Y(clusteroutput(d(l,1))))^2);
%         tl = tl + tlen;
%         else
%         tlen = sqrt((X(clusteroutput(d(l,j))) - X(clusteroutput(d(l,j+1))))^2 + (Y(clusteroutput(d(l,j))) - Y(clusteroutput(d(l,j+1))))^2);
%         tl = tl + tlen;
%         end
%     end
%     tlen1(l)= int32(tl);
% end
% 
% minid = 1;
% for l= 2:clusterpolPointCnt
%     if tlen1(l) <= tlen1(minid)
%         minid=l;
%     end
% end
% for l = 1:clusterpolPointCnt
%     disc(l) = d(minid,l);
% end
% % [disc, pred, closed] = graphtraverse(G1, 1,'Method','DFS');
% 
% if numel(disc)==1
%                     
% TlWC = sqrt((X(clusteroutput(disc(k))) - entrypointX)^2 + (Y(clusteroutput(disc(k))) - entrypointY)^2);
% disp(TlWC);
% Tourlength = Tourlength + TlWC;              
% line([X(clusteroutput(disc(k)))  entrypointX], [Y(clusteroutput(disc(k)))  entrypointY], 'LineStyle', '-', 'color','g');
% 
% TlWC = sqrt((X(clusteroutput(disc(k))) - exitpointX)^2 + (Y(clusteroutput(disc(k))) - exitpointY)^2);
% Tourlength = Tourlength + TlWC;
% line([X(clusteroutput(disc(k)))  exitpointX], [Y(clusteroutput(disc(k)))  exitpointY], 'LineStyle', '-', 'color','g');
% else                  
% for k = 0:clusterpolPointCnt
%     if k == clusterpolPointCnt
%         TlWC = sqrt((exitpointX - X(clusteroutput(disc(clusterpolPointCnt))))^2 + (exitpointY - Y(clusteroutput(disc(clusterpolPointCnt))))^2);
%         disp(TlWC);
%         Tourlength = Tourlength + TlWC;
%         line([entrypointX X(clusteroutput(disc(1)))], [entrypointY Y(clusteroutput(disc(1)))], 'LineStyle', '-', 'color','g');
%     else
%         if k == 0;
%             TlWC = sqrt((entrypointX - X(clusteroutput(disc(k+1))))^2 + (entrypointY - Y(clusteroutput(disc(k+1))))^2);
%             disp(TlWC);
%             Tourlength = Tourlength + TlWC;
%             line([exitpointX X(clusteroutput(disc(clusterpolPointCnt)))], [exitpointY Y(clusteroutput(disc(clusterpolPointCnt)))], 'LineStyle', '-', 'color','g');
%         else
%             TlWC = sqrt((X(clusteroutput(disc(k))) - X(clusteroutput(disc(k+1))))^2 + (Y(clusteroutput(disc(k))) - Y(clusteroutput(disc(k+1))))^2);
%             disp(TlWC);
%             Tourlength = Tourlength + TlWC;
%             line([X(clusteroutput(disc(k))) X(clusteroutput(disc(k+1)))], [Y(clusteroutput(disc(k))) Y(clusteroutput(disc(k+1)))], 'LineStyle', '-', 'color','g');
%         end
%     end
% end
% 
% end 
%  else
%     TlWC = sqrt((exitpointX - entrypointX)^2 + (exitpointY - entrypointY)^2);
%     disp(TlWC);
%     Tourlength = Tourlength + TlWC;
%     line([entrypointX exitpointX], [entrypointY exitpointY], 'LineStyle', '-', 'color','g');
% end
% else
%     ppoints = [ppoints,nodepoints];
%     Nmsg = [Nmsg, 1,1];
% 
%     TlWC = sqrt((X(pxy(1)) - X(pxy(2)))^2 + (Y(pxy(1)) - Y(pxy(2)))^2);
%     disp(TlWC);
%     Tourlength = Tourlength + TlWC;
%     line([X(pxy(1)) X(pxy(2))], [Y(pxy(1)) Y(pxy(2))], 'LineStyle', '-', 'color','g');

end
                    
point1 = [];
point2 = [];
pxy = [];
clusteroutput = [];
nodepoints = [];
    end   
end
x1cnt = numel(x1);
[ compmatrix, out ] = Complete_Mat( x1,y1 , x1cnt );
%%
[p,L] = tspsearch(out,1);
 rad = zeros(1,x1cnt);
%  rad =  rad + 15;
plot_TSP(x1cnt, x1, y1, CR, p);
 model = CreateModel(x1,y1);
tour = PSO_Tour(model);
psolen=tour.Cost;
disp(psolen);
%fireflies
v2=reshape(out,1,(model.n*2));
modfire1="main";
py.reload(py.importlib.import_module(modfire1));
modfire="fireflies";
py.reload(py.importlib.import_module(modfire));

firelen = py.main.find_flen(v2);
disp(firelen);
%cuckoo
x=model.D;
v1=reshape(x,1,model.n*model.n);
%system('python PCodes.our_modulep.Cuckoo_Length.find_len(v3)');
moduleour="Cuckoo_Length";
py.reload(py.importlib.import_module(moduleour));
cucklen = py.Cuckoo_Length.find_len(v1);
disp(cucklen);
%tsp-search
disp(L);
% end
fid=fopen('output.txt','a');
fprintf(fid,'\n\r');
fprintf(fid,'%d\t%d\t%d\t%d',L,psolen,cucklen,firelen);
fprintf(fid,'\n\r');
fclose(fid);
end