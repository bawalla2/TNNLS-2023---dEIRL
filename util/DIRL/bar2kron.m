% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAP \bar{x} -> kron(x, x)
%
% Brent Wallace  
%
% 2022-12-08
%
% This program, given a vector x \in R^{n} and the input argument \bar{x} =
% B(x, x) \in R^{n(n+1)/2} defined by
%
% \bar{x} = [x_1^2, x_1 x_2, ..., x_1 x_n, x_2^2, x_2 x_3, ..., x_2 x_n,
%               ... x_n^{2} ]
%
% returns the vector kron(x, x) \in R^{n^2}.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xkx = bar2kron(barx)

% Get length of input vector
nb = size(barx,1);

% Get underlying vector dimension
n = (sqrt(8*nb + 1) - 1) / 2;

% Get the matrix M \in R^{n^2 x n(n+1)/2} such that \bar{x} = M kron(x,x)
M = zeros(n^2,nb);
 
% Fill M
cnt = 1;
for i = 1:n
    for j = i:n
        indkron_12 = (i-1)*n + j;
        indkron_21 = (j-1)*n + i;
        M([indkron_12 indkron_21],cnt) = 1;
        cnt = cnt + 1;
    end
end

% Calculate output
xkx = M * barx;

