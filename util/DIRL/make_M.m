% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAKE KRONECKER TO BILINEAR FORM CONVERSION MATRIX\
%
% Brent Wallace  
%
% 2022-12-08
%
% This program makes the matrix M \in R^{n(n+1)/2 x n^2) such that
%
%       B(x, y) = M kron(x, y)
%
% and the right inverse M_{r}^{-1} of M such that
%
%       kron(x, x) = M_{r}^{-1} B(x, x)
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


function [Mn, Mrn]= make_M(n)

% Binomial coefficient
nb = n*(n+1)/2;

% Declare M, M_{r}^{-1} 
Mn = zeros(nb,n^2);
Mrn = zeros(n^2,nb);

% Fill M, M_{r}^{-1} 
cnt = 1;
for i = 1:n
    for j = i:n
        indkron_12 = (i-1)*n + j;
        indkron_21 = (j-1)*n + i;
        % Fill M
        Mn(cnt,indkron_12) = Mn(cnt,indkron_12) + 0.5;
        Mn(cnt,indkron_21) = Mn(cnt,indkron_21) + 0.5;
        % Fill M_{r}^{-1}
        Mrn([indkron_12 indkron_21],cnt) = 1;
        cnt = cnt + 1;
    end
end

