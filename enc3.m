% =======================================
%  begin subfunction
% =======================================
function [center,radius] = enc3(X,Y)
% minimum radius enclosing circle for exactly 3 points
%
% x, y are 3x1 vectors

% convert to complex
xy = X + sqrt(-1)*Y;

% just in case the points are collinear or nearly so, get
% the interpoint distances, and test the farthest pair
% to see if they work.
Dij = @(XY,i,j) abs(XY(i) - XY(j));
D12 = Dij(xy,1,2);
D13 = Dij(xy,1,3);
D23 = Dij(xy,2,3);

% Find the most distant pair. Test if their circumcircle
% also encloses the third point.
if (D12>=D13) && (D12>=D23)
  center = (xy(1) + xy(2))/2;
  radius = D12/2;
  if abs(center - xy(3)) <= radius
    center = [real(center),imag(center)];
    return
  end
elseif (D13>=D12) && (D13>=D23)
  center = (xy(1) + xy(3))/2;
  radius = D13/2;
  if abs(center - xy(2)) <= radius
    center = [real(center),imag(center)];
    return
  end
elseif (D23>=D12) && (D23>=D13)
  center = (xy(2) + xy(3))/2;
  radius = D23/2;
  if abs(center - xy(1)) <= radius
    center = [real(center),imag(center)];
    return
  end
end

% if we drop down to here, then the points cannot
% be collinear, so the resulting 2x2 linear system
% of equations will not be singular.
A = 2*[X(2)-X(1), Y(2)-Y(1); X(3)-X(1), Y(3)-Y(1)];
rhs = [X(2)^2 - X(1)^2 + Y(2)^2 - Y(1)^2; ...
       X(3)^2 - X(1)^2 + Y(3)^2 - Y(1)^2];
     
center = (A\rhs)';
radius = norm(center - [X(1),Y(1)]);