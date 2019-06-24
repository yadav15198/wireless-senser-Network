function plot_TSP(Nnum, X, Y, radius, INTIAL_TSP)
fig_cnt = 1;
figure(fig_cnt);
rectangle('Position', [0,0,600,600]);
hold on;
for i=1:Nnum
    plot(X(i),Y(i),'.','MarkerSize',10);
    text(X(i),Y(i),num2str(i),'fontsize',12);
    circle(X(i),Y(i),radius);
    hold on;
end
for i=1:Nnum
    if (i+1) <= Nnum
    p1=[X(INTIAL_TSP(i)),Y(INTIAL_TSP(i))];
    p2=[X(INTIAL_TSP(i+1)),Y(INTIAL_TSP(i+1))];
    dp=p2-p1;
    quiver(p1(1),p1(2),dp(1),dp(2),0);
    else
        if i == Nnum
            p1=[X(INTIAL_TSP(Nnum)),Y(INTIAL_TSP(Nnum))];
            p2=[X(INTIAL_TSP(1)),Y(INTIAL_TSP(1))];
            dp=p2-p1;
            quiver(p1(1),p1(2),dp(1),dp(2),0);
        end
    end    
end

fig_cnt = fig_cnt + 1;
end
