% find a tsp tour on polling points
function[length,order]= tsp(completegraph,points)
Z = numel(points);
order = points(1);
distarray = [];
length = 0;

for i = 1;
    for j = 1: Z
        if i == j
            continue;
        else
            distarray = [distarray,completegraph(i,j)];
        end
    end
end

minid = 1;
for i = 2:numel(distarray)
   if distarray(i) <= distarray(minid)
        minid=i;
   end 
end 
covered = zeros(1,numel(points));
for i = 1:Z
    if i == minid
        covered(i) = 1;
    end
end
order = [order,points(minid)];
length = length + distarray(minid);
uncovered = numel(points) - numel(order);
while uncovered > 0
    for i = minid;
        for j = 1: numel(points)
            if covered == 1
                continue;
            else
            distarray = [distarray,completegraph(i,j)];
            end
        end
    end

minid = 1;
for i = 2:numel(distarray)
    if distarray(i) <= distarray(minid)
        minid=i;
    end
end
end
end