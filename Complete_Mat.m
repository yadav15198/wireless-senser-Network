function [ compgraph,out ] = Complete_Mat( X, Y, N )
compgraph=zeros(N,N);
for i=1:N
    for j=1:N
        distance = sqrt((X(i)-X(j))^2 + (Y(i)-Y(j))^2);
        compgraph(i,j)=distance;
    end
end
% out = randi(500,N,2);
% % [r,c] = size(out);
for i = 1 : N
    out(i,1) = X(i);
    out(i,2) = Y(i);
end    
end

