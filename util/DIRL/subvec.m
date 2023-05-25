% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% GET SUBSET OF KRONECKER PRODUCT
%
% Brent Wallace  
%
% 2022-12-08
%
% Given x \in R^{n}, u \in R^{m}, with 1 <= p <= n, 1 <= q <= m, Given
% the index sets I_x \subset {1,...,n}^p and I_u \subset {1,...,m}^q and
% the vector kron(x,u) \in R^{nm}, this program returns the vector 
%
% kron(x(I_x), u(I_u))
%
% Where
%
% x(I_x) = [x_{I_{x,1}}, ..., x_{I_{x,p}}]^T \in R^{p}
%
% u(I_u) = [u_{I_{u,1}}, ..., u_{I_{u,q}}]^T \in R^{q}
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xpup = subvec(xu, n, m, Ix, Iu)

% Get size of index vectors
p = size(Ix,1);
q = size(Iu,1);

% Number of columns in input matrix
ncol = size(xu,2);

% Initialize empty output
xpup = zeros(p*q, ncol);

% Form each column of output matrix
for j = 1:ncol

    % Tabulate xu(:,j) as matrix
    xum = reshape(xu(:,j),[m,n]);

    % Get output
    xpum = xum(:,Ix);
    xpupm = xpum(Iu,:);
    xpup(:,j) = xpupm(:);

end




