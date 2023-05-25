% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% VECTORIZE SYMMETRIC MATRIX
%
% Brent Wallace  
%
% 2022-12-08
%
% Given a symmetric matrix P \in R^{n \times n}, this program returns the
% vector v(P) \in R^{n(n+1)/2} which is the vectorization of P obtained by
% taking its row-wise elements above the diagonal, wherein elements on the
% diagonal are multiplied by 1 and elements above the diagonal are
% multiplied by 2.
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function vP = vecsym(P)

% Size of input matrix P
n = size(P,1);

% Initialize empty output vector
vP = [];

for i = 1:n

    % Get elements of row i of P in the diagonal and above
    vPi = P(i,i:n)';

    % Multiply entries 2:n by the constant 2
    vPi(2:end) = 2 * vPi(2:end);

    % Append this entry to the vectorization
    vP = [  vP
            vPi     ];

end






