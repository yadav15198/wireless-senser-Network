% implementation of tour length for spanning tree covering algorithm(SHDGP)

clear;
close;

noofiteration = 2;

costarray = zeros(1,noofiteration);         % containing cost value for each iteration of SHDGP
clustercostarray = zeros(1,noofiteration);  % containing cost value for each iteration of CBSHDGP
clusterpolpointsnum = zeros(1,noofiteration);  % containing number of polling points in each iteration

% network lifetime (50%, 90%, 100% )for SHDGP

netlftime50 = zeros(1,noofiteration);
netlftime90 = zeros(1,noofiteration);
netlftime100 = zeros(1,noofiteration);

% network lifetime (50%, 90%, 100% )for CBSHDGP

Cnetlftime50 = zeros(1,noofiteration);
Cnetlftime90 = zeros(1,noofiteration);
Cnetlftime100 = zeros(1,noofiteration);

for k = 1:noofiteration
    
close;

N=50;                  % number of sensors
CR = 50;                %communication range

startX=0;
startY=250;

curPosX=startX;
curPosY=startY;



NpollingPoints=N;                               % number of polling points

adjMatrix=zeros(N,N);                           % containing distance value
Covered=zeros(1,N);                             % sensors which are covered by the mobile collectors
CoveredPollingPoints= zeros(1,NpollingPoints);  % polling points which are traversed by the mobile collectors

polpointX=zeros(1,NpollingPoints);
polpointY=zeros(1,NpollingPoints);

output=[];                                      % store the ordering of the polling points
NofMSG = [];                                    % Number of msg pass by each sensor (polling point)

figure(1);
rectangle('Position', [0,0,300,300])
hold on;

%% Random Input generator in terms of X and Y

X = randi(600,N,1); 
Y = randi(600,N,1);

polpointX=X;
polpointY=Y;

% plot all candidate polling point

for i=1:N
plot(X(i),Y(i),'o','MarkerSize',10);
text(X(i),Y(i), num2str(i),'fontsize',10);
end

% Finding the distance to each polling points from mobile collector

pptomcdist = zeros(1,NpollingPoints); % Distance from mobile collectors to othere polling points

for i=1:NpollingPoints
    pptomcdist(i) = sqrt(   (curPosX - X(i))^2 + (curPosY - Y(i))^2   );
end


xpos=0;
ypos=0;

ns = zeros(NpollingPoints,N);                   % Neighbor set
nns = zeros(1,NpollingPoints);                  % Number of neighbors

costOfPollingPoints= zeros(1,NpollingPoints);   % Cost of polling points from the mobile collector

% Calcullating the neighbor set of each polling points

for i = 1:NpollingPoints
    
    for j = 1:N
        distance = sqrt((polpointX(i) - X(j))^2 + (polpointY(i) - Y(j))^2);
        adjMatrix(i,j)=distance;
        if distance <= CR
            line([polpointX(i) polpointX(j)], [polpointY(i) polpointY(j)], 'LineStyle', ':');
            ns(i,j) = 1;
            nns(i)=nns(i)+1;
        end
      
    end
end

% Finding the cost of each polling point from the mobile collector


for i=1:NpollingPoints

       costOfPollingPoints(i)= pptomcdist(i)/nns(i);
end


% Find the minimum cost polling points

minIndx=1;

for i=2:NpollingPoints
    
    if costOfPollingPoints(i) <= costOfPollingPoints(minIndx)
        minIndx=i;
    end
    
end
 
output=[output,minIndx];
NofMSG = [NofMSG,nns(minIndx)];
CoveredPollingPoints(minIndx)=1;

% sensors covered by the polling point

for i=1:N
    if ns(minIndx,i)==1
        Covered(i)=1;
    end
end

curPosX=polpointX(minIndx);
curPosY=polpointY(minIndx);

nUncovered= N-nns(minIndx);


while nUncovered > 0
    
    curPosX=polpointX(minIndx);
    curPosY=polpointY(minIndx);

% Find the distance to the remaining polling points
    
                        % % % % %                     %     for i=1:NpollingPoints
                        % % % % %                     %         if Covered(i)==1
                        % % % % %                     %             d(i)=inf;
                        % % % % %                     %             nns(i) = inf;
                        % % % % %                     %             costOfPollingPoints(i) = inf;
                        % % % % %                     %         else
                        % % % % %                     %             d(i) = sqrt(   (curPosX - X(i))^2 + (curPosY - Y(i))^2   );
                        % % % % %                     %         end
                        % % % % %                     %     end

for i=1:NpollingPoints
    min = 0;
    if Covered(i)==1
        pptomcdist(i)=inf;
        nns(i) = inf;
        costOfPollingPoints(i) = inf;
    else
        for j = 1:numel(output)
            if min == 0
                min = sqrt ( ( X(output(j))- X(i))^2 +(Y(output(j))-Y(i))^2);
            end

            temp = sqrt ( ( X(output(j))- X(i))^2 +(Y(output(j))-Y(i))^2);
            if temp <= min
                min = temp;
                pptomcdist(i) = min;
            end
        end
    end
end

% Update neighbor set and number of neighbors of polling points

    for i=1:NpollingPoints
       
        if CoveredPollingPoints(i)==1
            continue;
        end
        
        for j = 1:N
            if ns(i,j)==1 && Covered(j)
                  ns(i,j) = 0;
                  nns(i)=nns(i)-1;
            end

       end 
    end
    
    
% Finding the cost of each polling point from the mobile collector


    for i=1:NpollingPoints
        if costOfPollingPoints(i)==inf
            continue;
        end
        costOfPollingPoints(i)= pptomcdist(i)/nns(i);
    end


% Find the minimum cost polling points

%     minIndx=1;

    for i=1:NpollingPoints
        
        if costOfPollingPoints(i) <= costOfPollingPoints(minIndx)
            minIndx=i;
        end

    end


    output=[output,minIndx];
    NofMSG = [NofMSG,nns(minIndx)];

    CoveredPollingPoints(minIndx)=1;
    
% sensors covered by the polling point

    for i=1:N
        if ns(minIndx,i)==1
            Covered(i)=1;
        end
    end

 nUncovered= nUncovered-nns(minIndx);

end

%finding approximation tour

polPointCnt = numel(output);
outmat = zeros(numel(output),numel(output));

for i= 1:polPointCnt
    for j= 1:polPointCnt
        dist = sqrt((X(output(i)) - X(output(j)))^2 + (Y(output(i)) - Y(output(j)))^2);
        outmat(i,j) = dist;
    end
end


G = sparse(outmat);
[Tree, pred1]=graphminspantree(G);
mst2 = full(Tree);
 
%% representation of mst
for i = 1:polPointCnt
    for j = 1:polPointCnt
        if mst2(i,j)~= 0  
            xpos = i;
            ypos = j;
            line([X(output(xpos)) X(output(ypos))], [Y(output(xpos)) Y(output(ypos))], 'LineStyle', '-', 'color','r');
        end
    end
end
%%

dfsmat = zeros(numel(output),numel(output));

for i= 1:polPointCnt
    for j= 1:polPointCnt
        dfsmat(i,j) = mst2(i,j);
        dfsmat(j,i) = mst2(i,j);
    end
end

%% perform DFS and find the order of node to visit with minimum cost

d= zeros(polPointCnt,polPointCnt);
G1 = sparse(dfsmat);

[disc] = graphtraverse(G1, 1,'Method','DFS');

% for i = 1:polPointCnt
% [order] = graphtraverse(G1, i,'Method','DFS');
% for j = 1 : polPointCnt
%     d(i,j) = order(j);
% end    
% end

% tlen1 = zeros(1,polPointCnt);
% 
% for i = 1:polPointCnt
%     tl = 0;
%     for j = 1:polPointCnt
%         if j == polPointCnt
%         tlen = sqrt((X(output(d(i,j))) - X(output(d(i,1))))^2 + (Y(output(d(i,j))) - Y(output(d(i,1))))^2);
%         tl = tl + tlen;
%         else
%         tlen = sqrt((X(output(d(i,j))) - X(output(d(i,j+1))))^2 + (Y(output(d(i,j))) - Y(output(d(i,j+1))))^2);
%         tl = tl + tlen;
%         end
%     end
%     tlen1(i)= int32(tl);
% end
% 
% minid = 1;
% for i= 2:polPointCnt
%     if tlen1(i) <= tlen1(minid)
%         minid=i;
%     end
% end
% for i = 1:polPointCnt
%     disc(i) = d(minid,i);
% end
%%
visitorder = [];
for i = 1:polPointCnt
    visitid = output(disc(i));
    visitorder = [visitorder,visitid];
end

% network lifetime

[ nwlftime50,  nwlftime90, nwlftime100 ] = Networklifetime(N, X, Y, output, NofMSG, visitorder);
netlftime50(k) = nwlftime50;
netlftime90(k) = nwlftime90;
netlftime100(k) = nwlftime100;

% G1 = sparse(dfsmat);
% [disc, pred, closed] = graphtraverse(G1, 1,'Method','DFS');
% order = graphtraverse(G,1);

cost = 0;
for i = 1:numel(output)
    if i==numel(output)
        value = sqrt((X(output(disc(1))) - X(output(disc(numel(output)))))^2 + (Y(output(disc(1))) - Y(output(disc(numel(output)))))^2);
        cost = cost + value;
        line([X(output(disc(1))) X(output(disc(numel(output))))], [Y(output(disc(1))) Y(output(disc(numel(output))))], 'LineStyle', '-', 'color','g');
    else
    value = sqrt((X(output(disc(i))) - X(output(disc(i+1))))^2 + (Y(output(disc(i))) - Y(output(disc(i+1))))^2);
    line([X(output(disc(i))) X(output(disc(i+1)))], [Y(output(disc(i))) Y(output(disc(i+1)))], 'LineStyle', '-', 'color','g');
    cost = cost + value;
    end
end 
cost1 = int32(cost);
disp(cost1);
costarray(k) = cost1;

% finding the tour cost of disconnected network form from N sensors

% [tour, mintree, tourcost]   = optimaltour(N,X,Y, CR);
% clustercostarray(k) = tourcost;

[polpoints, opttour, clustermst, totcos, Cnwlftime50, Cnwlftime90, Cnwlftime100] = optour(N,X,Y,CR);
disp('optour');
disp(totcos);
clustercostarray(k) = totcos;
clusterpolpointsnum(k) = numel(polpoints);
Cnetlftime50(k) = Cnwlftime50;
Cnetlftime90(k) = Cnwlftime90;
Cnetlftime100(k) = Cnwlftime100;
[ppoint, opttour1, clustermst1, totcost] = neighborbydistance(N,X,Y,CR);
disp('nbydist =');
disp(totcost);
end

% avgcost = mean(costarray);
% disp('The avgcost to gather all the data within onehop distance');
% disp(int32(avgcost));
% 
% avgtourcost = mean(clustercostarray);
% disp('Avgtour cost to visit all the clusters.');
% disp(int32(avgtourcost));
% 
% avgnumofpolpoints = mean(clusterpolpointsnum);
% disp('Avgtour Number of polling points in CBSHDGA.');
% disp(int32(avgnumofpolpoints));
% 
% avgnetlftime50 = mean(netlftime50);
% disp('50 % nettime');
% disp(int32(avgnetlftime50));
% 
% avgnetlftime90 = mean(netlftime90);
% disp('90 % nettime');
% disp(int32(avgnetlftime90));
% 
% avgnetlftime100 = mean(netlftime100);
% disp('100 % nettime');
% disp(int32(avgnetlftime100));
% 
% Cavgnetlftime50 = mean(Cnetlftime50);
% disp('50 % Cnettime');
% disp(int32(Cavgnetlftime50));
% 
% Cavgnetlftime90 = mean(Cnetlftime90);
% disp('90 % Cnettime');
% disp(int32(Cavgnetlftime90));
% 
% Cavgnetlftime100 = mean(Cnetlftime100);
% disp('100 % Cnettime');
% disp(int32(Cavgnetlftime100));



