% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% OBTAIN SYMMETRIC MATRIX FROM VECTORIZATION
%
% This program is the inverse operation of the one above. I.e., given a
% vectorization v(P) \in R^{n(n+1)/2} as described above, this program
% returns the symmetric matrix P \in R^{n \times n} associated with it.
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function P = invvecsym(vP)

% Length of input vector
lvP = size(vP,1);

% Get size of input matrix P
n = (sqrt(8*lvP + 1) - 1) / 2;

% Initialize empty output matrix
P = zeros(n);

% Initialize an accumulative index count
cnt = 1;

for i = 1:n

    % Indices of v(P) to extract
    indsi = (cnt:cnt+(n-i))';

    % Get i-th row of P in the diagonal and above (scaled)
    vPi = vP(indsi);

    % Apply inverse scaling
    vPi(2:end) = 1/2 * vPi(2:end);

    % Fill in corresponding row, column of P
    P(i,i:n) = vPi;
    P(i:n,i) = vPi;

    % Increment accumulator
    cnt = cnt + size(indsi,1);

end




