% implementation of tour length of single M-collector

clear;
close;

N=200;   % number of sensors
CR = 60; %communication range
NpollingPoints=N;  % number of polling points

startX=0;
startY=250;


curPosX=startX;
curPosY=startY;


Covered=zeros(1,N); % sensors which are covered by the mobile collectors
CoveredPollingPoints= zeros(1,NpollingPoints); % polling points which are traversed by the mobile collectors
adjMatrix=zeros(N,N);
output=[]; % store the ordering of the polling points
NofMSG = []; %number of msg pass in each tour bypolling point

% Initial Graph Construction with Mobile Projector Path

% fid = fopen('sample2.txt','r');
% X=fscanf(fid,'%d',N);
% Y=fscanf(fid,'%d',N);
% fclose(fid);

X = randi(500,N,1); 
Y = randi(500,N,1);
% X = [54;481;3;388;409;435;43;200;130;401;216;456;91;132;73;69;435;290;275;73;427;312;176;257;201;38;120;62;92;120];
% Y = [209;25;452;473;246;245;169;451;185;56;391;195;121;202;49;66;472;479;288;30;118;177;411;8;22;85;325;366;324;226];
% [72;280;3;384;425;459;494;253;136;51;254;293;382;42;331;259;86;470;296;221];
% [471;328;226;420;267;277;341;184;120;290;434;204;57;222;151;201;417;202;196;181];

% X = [50, 150, 100, 50, 90, 100, 75, 125, 175, 165, 210, 250, 225, 250, 275, 155];
% Y = [300, 300, 250, 200, 200, 150, 100, 90, 100, 165, 185, 125, 250, 300, 225, 240];

figure(1);
rectangle('Position', [0,0,500,500])
hold on;

polpointX=X;
polpointY=Y;


% plot all candidate polling point
for i=1:N
plot(X(i),Y(i),'o','MarkerSize',10);
text(X(i),Y(i), num2str(i),'fontsize',10);
end

% selecting the first polling point

% Finding the distance to other polling points from mobile collector

d = zeros(1,NpollingPoints); % Distance from mobile collectors to othere polling points
ns = zeros(NpollingPoints,N);   % Neighbor set
nns = zeros(1,NpollingPoints);  % Number of neighbors
costOfPollingPoints= zeros(1,NpollingPoints); % Cost of polling points from the mobile collector


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

% Calculate the distance of the polling points from the mobile sink's current position

for i=1:NpollingPoints
    d(i) = sqrt(   (curPosX - polpointX(i))^2 + (curPosY - polpointY(i))^2   );
end

% Finding the cost of each polling point from the mobile collector
             
for i=1:NpollingPoints
    
    costOfPollingPoints(i)= d(i)/nns(i);
    
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
    
for i=1:NpollingPoints
    min = 0;
    if Covered(i)==1
        d(i)=inf;
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
                d(i) = min;
%            d(i) = sqrt(   (curPosX - X(i))^2 + (curPosY - Y(i))^2   );
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
        costOfPollingPoints(i)= d(i)/nns(i);
    end


% Find the minimum cost polling points

%     minIndx=1;

    for i=1:NpollingPoints
        
        if costOfPollingPoints(i) <= costOfPollingPoints(minIndx)
            minIndx=i;
        end

    end
    
    output=[output, minIndx];
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



polPointCnt= numel(output); % Count the number of polling points

outmat = zeros(polPointCnt, polPointCnt);

for i= 1:polPointCnt
    for j= 1:polPointCnt
        dist = sqrt((X(output(i)) - X(output(j)))^2 + (Y(output(i)) - Y(output(j)))^2);
        outmat(i,j) = dist;
    end
end
   

G = sparse(outmat);
[Tree]=graphminspantree(G);
mst1 = full(Tree);

%% representation of mst
for i = 1:numel(output)
    for j = 1:numel(output)
        if mst1(i,j)~= 0  
            xpos = i;
            ypos = j;
            line([X(output(xpos)) X(output(ypos))], [Y(output(xpos)) Y(output(ypos))], 'LineStyle', '-', 'color','r');
        end
    end
end
%%

dfsmat = zeros(polPointCnt,polPointCnt);

for i= 1:polPointCnt
    for j= 1:polPointCnt
        dfsmat(i,j) = mst1(i,j);
        dfsmat(j,i) = mst1(i,j);
    end
end

%% perform DFS and find the order of node to visit with minimum cost

d= zeros(polPointCnt,polPointCnt);
G1 = sparse(dfsmat);
for i = 1:polPointCnt
[order] = graphtraverse(G1, i,'Method','DFS');
for j = 1 : polPointCnt
    d(i,j) = order(j);
end    
end

tlen1 = zeros(1,polPointCnt);

for i = 1:polPointCnt
    tl = 0;
    for j = 1:polPointCnt
        if j == polPointCnt
        tlen = sqrt((X(output(d(i,j))) - X(output(d(i,1))))^2 + (Y(output(d(i,j))) - Y(output(d(i,1))))^2);
        tl = tl + tlen;
        else
        tlen = sqrt((X(output(d(i,j))) - X(output(d(i,j+1))))^2 + (Y(output(d(i,j))) - Y(output(d(i,j+1))))^2);
        tl = tl + tlen;
        end
    end
    tlen1(i)= int32(tl);
end

minid = 1;
for i= 2:polPointCnt
    if tlen1(i) <= tlen1(minid)
        minid=i;
    end
end
for i = 1:polPointCnt
    disc(i) = d(minid,i);
end
%%
visitorder = [];
for i = 1:polPointCnt
    visitid = output(disc(i));
    visitorder = [visitorder,visitid];
end

% network lifetime
[ nwlftime50,  nwlftime90, nwlftime100 ] = Networklifetime(N, X, Y, output, NofMSG, visitorder);

Tourlength = 0;

for i = 1:polPointCnt
    if i==polPointCnt
        Tl = sqrt((X(output(disc(1))) - X(output(disc(polPointCnt))))^2 + (Y(output(disc(1))) - Y(output(disc(polPointCnt))))^2);
        Tourlength = Tourlength + Tl;
        line([X(output(disc(1))) X(output(disc(polPointCnt)))], [Y(output(disc(1))) Y(output(disc(polPointCnt)))], 'LineStyle', '-', 'color','b');
    else
    Tl = sqrt((X(output(disc(i))) - X(output(disc(i+1))))^2 + (Y(output(disc(i))) - Y(output(disc(i+1))))^2);
%     disp(Tl);
    Tourlength = Tourlength + Tl;
    line([X(output(disc(i))) X(output(disc(i+1)))], [Y(output(disc(i))) Y(output(disc(i+1)))], 'LineStyle', '-', 'color','b');
   end
end
     
disp(int32(Tourlength));

% [tour, mintree, tourcost]   = optimaltour1(N,X,Y, CR);












% % for i = 1:polPointCnt
% %     for j =1:polPointCnt
% %        if dfsmat(i,j) == mst1 (i,j) && dfsmat(i,j) ~= 0 && mst1 (i,j) ~= 0
% % %        dfsmat(j,i) ~=0    
% %         value = dfsmat(i,j);
% %         disp(value);
% %         Tourlength = Tourlength + value;
% %         line([X(output(i)) X(output(j))], [Y(output(i)) Y(output(j))], 'LineStyle', '_', 'color','b');
% % %         line([X(output(disc(i))) X(output(disc(j)))], [Y(output(disc(i))) Y(output(disc(j)))], 'LineStyle', '-', 'color','r');
% %        end
% %     end
% %     
% %      
% % end




% % %        if dfsmat(i,j) == mst1(i,j) && dfsmat(i,j) == mst1(j,i) && dfsmat(i,j) ~= 0 && dfsmat(j,i) ~=0
% % %         value = dfsmat(i,j);




%%
% % % representation of mst
% % 
% %  for i = 1:polPointCnt-1;
% %     line([X(output(Tree(i,1))) X(output(Tree(i,2)))], [Y(output(Tree(i,1))) Y(output(Tree(i,2)))], 'LineStyle', '-', 'color','r');
% %  end