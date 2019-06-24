
function h = circle(X,Y,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + X;
yunit = r * sin(th) + Y;
h = plot(xunit, yunit);
hold off