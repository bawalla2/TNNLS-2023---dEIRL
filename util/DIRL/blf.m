% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE BILINEAR FORM OF TWO VECTORS
%
% Brent Wallace  
%
% 2022-12-08
%
% Given two vectors x, y \in R^{n}, this program calculates the vector
% B(x,y) \in R^{n(n+1)/2} given by
%
% B(x,y) = 1/2 * [  2 x_1^2
%                   x_1 y_2 + x_2 y_1
%                   x_1 y_3 + x_3 y_1
%                   ...
%                   x_1 y_n + x_n y_1
%                   2 x_2 y_2
%                   x_2 y_3 + x_3 y_2
%                   ...
%                   x_2 y_n + x_n y_2
%                   ...
%                   2 x_n y_n             ]
%
% 
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function Bxy = blf(x, y)

% Length of input vector
n = size(x,1);

% Initialize empty output
Bxy = zeros(n*(n+1)/2, 1);

% Initialize an accumulative index count
cnt = 1;

for i = 1:n

    for j = i:n
        Bxy(cnt) = 1/2 * (x(i) * y(j) + x(j) * y(i));
        cnt = cnt + 1;
    end

end


