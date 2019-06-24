function [nwlftime50, nwlftime90, nwlftime100] = Networklifetime(N, X, Y, polpoint, NMsg, visitorder)

% Network lifetime of the network 

MCS = 1; % 1m/s
IE = 10;
d = zeros(1,numel(polpoint));
for i = 1:numel(polpoint)
    if i == numel(polpoint)
        d(i) = sqrt((X(visitorder(i)) - X(visitorder(1)))^2 + (Y(visitorder(i)) - Y(visitorder(1)))^2);
    else
      d(i) = sqrt((X(visitorder(i)) - X(visitorder(i+1)))^2 + (Y(visitorder(i)) - Y(visitorder(i+1)))^2); 
    end
end

% calculation of networklifetime

time = 0;
deadnode = 0;
RE = zeros(1,numel(polpoint)); % remaning energy matrix
for i = 1:numel(polpoint)
for j = 1:numel(polpoint)
    if visitorder(i) == polpoint(j)
        RE(i) = IE-NMsg(i);
        time  = time + d(i)/MCS; % time in second
        if RE(i) <= 0;
            deadnode = deadnode + NMsg(i);
        end
    end
end
end
time50 = time;
while deadnode < N/2;
    for i = 1:numel(polpoint)
    for j = 1:numel(polpoint)
        if visitorder(i) == polpoint(j)
            if RE(i)<=0
                RE(i)=inf;
            end
            RE(i) = RE(i)-NMsg(i);
            time50  = time50 + d(i)/1; % time in second
            if RE(i) <= 0;
                deadnode = deadnode + NMsg(i);
            end
            
        end
    end
        if deadnode >= N/2;
            break;
        end
        
    end
end

% 90% network lifetime

time90 = time50;
while deadnode < 9*N/10;
    for i = 1:numel(polpoint)
    for j = 1:numel(polpoint)
        if visitorder(i) == polpoint(j)
            if RE(i)<=0
                RE(i)=inf;
            end
            RE(i) = RE(i)-NMsg(i);
            time90  = time90 + d(i)/1; % time in second
            if RE(i) <= 0;
                deadnode = deadnode + NMsg(i);
            end
            
        end
    end
        if deadnode >= 9*N/10;
            break;
        end
    end
end

% 100% network lifetime

time100 = time90;
while deadnode < N;
    for i = 1:numel(polpoint)
    for j = 1:numel(polpoint)
        if visitorder(i) == polpoint(j)
            if RE(i)<=0
                RE(i)=inf;
            end
            RE(i) = RE(i)-NMsg(i);
            time100  = time100 + d(i)/1; % time in second
            if RE(i) <= 0;
                deadnode = deadnode + NMsg(i);
            end
            
        end
    end
        if deadnode >= N;
            break;
        end
    end
end
 nwlftime50 = int32(time50);
 nwlftime90 = int32(time90);
 nwlftime100 = int32(time100);
end

