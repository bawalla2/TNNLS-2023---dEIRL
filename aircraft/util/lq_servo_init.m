function lq_data = lq_servo_init(alg_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PERFORM LQ SERVO DESIGN
%
% Brent Wallace  
%
% 2023-03-29
%
% This program performs an LQ servo design from input model data. LQ servo
% performs integral augmentation at the plant output before doing an LQR
% design.
%
% REQUIRED PLANT STATE PARTITION:
%
%   x_p = [     y
%               x_r ]
%
% where x_p \in R^n is the plant state, y \in R^m is the plant output, and
% x_r \in R^{n-m} is the rest of the state (possibly empty).
%
% AUGMENTED PLANT STATE PARTITION:
%
%   x = [   z
%           x_p     ]
%
% where z \in R^m is are the integrator states at the plant output.
%
% CONTROLLER STATE PARTITION:
%
%   K = [ K_z   K_y     K_r ]   \in R^{m x (m+n)}
%
% Where K_z \in R^{m x m} is the integrator state feedback matrix, K_y \in
% R^{m x m} is the output state feedback matrix, and K_r \in R^{m x (n-m)}
% is the rest of the state feedback matrix.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%       
% Ap                    (n x n Matrix) Plant "A" matrix
% Bp                    (n x m Matrix) Plant "B" matrix
% Cp                    (m x n Matrix) Plant "C" matrix
%                       NOTE: For LQ servo, Cp must have the form:
%                           Cp = [I 0]
% Dp                    (m x m Matrix) Plant "D" matrix
% Q                     (n x n Matrix) State penalty matrix Q = Q^T >= 0
% R                     (m x m Matrix) Control penalty matrix R = R^T > 0
% su                    (m x m Matrix) Control coordinate transformation
% sx                    (n x n Matrix) State coordinate transformation
% sy                    (m x m Matrix) Output coordinate transformation
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% K                     (m x (m+n) Matrix) Composit LQ servo controller.
%                       Has form described above.
% K_z                   (m x m Matrix) Integrator state feedback matrix.
% K_y                   (m x m Matrix) Output state feedback matrix.
% K_r                   (m x (n-m) Matrix) Rest of state feedback matrix.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% UNPACK SETTINGS/PARAMETERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Plant state-space matrices
Ap = alg_settings.Ap;
Bp = alg_settings.Bp;
Cp = alg_settings.Cp;
Dp = alg_settings.Dp;

% Get system dimensions
n = size(Ap,1);
m = size(Bp,2);

% Get Q, R
Q = alg_settings.Q;
R = alg_settings.R;

% Get coord transformations
su = alg_settings.su;
sx = alg_settings.sx;
sy = alg_settings.sy;

% Data saving settings
savedata = isfield(alg_settings, 'relpath_data');

% If data is to be saved, extract the relative path and file name to save
% to
if savedata
    relpath_data = alg_settings.relpath_data;
    filename = alg_settings.filename;
end

%%
% *************************************************************************
% *************************************************************************
%
% FORM DESIGN PLANT
%
%
% Given an original plant of the form P = (A, B, C, D), we would like to
% form a design plant (appropriately scaled) such that the state vector
% assumes the form
%
% x' = [    y
%           x_r     ]
%
% Where y \in R^m are the outputs to be tracked, and x_r \in R^{n-m} forms
% the rest of the state.
%
% Introduce the coordinate transformations:
%
%  u' = S_u * u
%  x' = S_x * x
%  y' = S_y * y
%
% *************************************************************************
% *************************************************************************

% Scaled linear dynamics
Ad = sx * Ap * inv(sx);
Bd = sx * Bp * inv(su);
Cd = sy * Cp * inv(sx);
Dd = sy * Dp * inv(su);

% Scaled design plant
Pd = ss(Ad,Bd,Cd,Dd);

% Rest of state x_r = C_r x'
Cr = [zeros(n-m,m)  eye(n-m)];


% *************************************************************************
%
% AUGMENT PLANT WITH INTEGRATORS
%
% For zero steady-state error to step reference commands (internal model
% principle)
%
% State: [  z
%           x' ]
%
%      = [  z
%           y
%           x_r ]
%
% *************************************************************************


% ***********************
%       
% INTEGRATOR AUGMENTATION STATE SPACE REPRESENTATION
%
% State: z \in R^{2}
%   

Ai = zeros(m);
Bi = eye(m);
Ci = eye(m);
Di = zeros(m);

% ***********************
%       
% FORM AUGMENTED PLANT STATE SPACE
%  

Aaug = [    Ai          Bi * Cd
            zeros(n,m)  Ad        ];

Baug = [    Bi * Dd
            Bd          ];


% *************************************************************************
%       
% PERFORM LQR DESIGN
%  
% *************************************************************************

% Call LQR function
[K, P, ~] = lqr(Aaug, Baug, Q, R);

% Extract parts of feedback gain matrix K corresponding to each of the
% states in the partition
Kz = K(:,1:m);
Ky = K(:,m+1:2*m);
Kr = K(:,2*m+1:m+n);


% *************************************************************************
%       
% FORM OPEN-LOOP SYSTEM STATE SPACE REPRESENTATIONS
%  
% *************************************************************************


% ***********************
%       
% FORM LOOP BROKEN AT ERROR L_e
%
% State: [  z
%           x' ]
%
% Input: e
%
% Output: y
%  

% % z = \int -y
% Ae = [  Ai      zeros(m,n)
%         Bd*Kz  Ad - Bd*Kr*Cr ];
% % z = \int -y
% Be = [  Bi
%         Bd*Ky   ];

% z = \int y
Ae = [  Ai      zeros(m,n)
        -Bd*Kz  Ad - Bd*Kr*Cr ];
% z = \int y
Be = [  -Bi
        Bd*Ky   ];

Ce = [  zeros(m)    Cd];

De = zeros(m);

Le = ss(Ae,Be,Ce,De);

% aol = [ 0*ones(2,2) 0*ones(2,4)
%        -bp*gz   ap-bp*[0*ones(2,2) gr] ]
% 
% bol = [ - eye(2,2)
%           bp*gy        ]
% 
% col = [ 0*ones(2,2) cp ]
%        
% dol = 0*ones(2,2)


% ***********************
%       
% FORM LOOP BROKEN AT CONTROLS L_u = G_{LQ}
%
% State: [  z
%           x' ]
%
% Input: u_p
%
% Output: u
%  

Au = Aaug;
Bu = Baug;
Cu = K;
Du = zeros(m);

Lu = ss(Au,Bu,Cu,Du);


% *************************************************************************
%       
% FORM CLOSED-LOOP SYSTEM STATE SPACE REPRESENTATIONS
%  
% *************************************************************************

% ***********************
%       
% FORM CLOSED-LOOP DYNAMICS BROKEN AT ERROR L_e
%

Aecl = Ae - Be * Ce;
Becl = Be;
Cecl = Ce;
Decl = De;

% Sensitivity at the error S_e
Se = ss(Aecl, Becl, -Cecl, eye(m) - Decl);

% Comp sensitivity at error T_e
Te = ss(Aecl, Becl, Cecl, Decl);


% ***********************
%       
% FORM CLOSED-LOOP DYNAMICS BROKEN AT CONTROL L_u
%

Aucl = Au - Bu * Cu;
Bucl = Bu;
Cucl = Cu;
Ducl = Du;

% Sensitivity at the control S_u
Su = ss(Aucl, Bucl, -Cucl, eye(m) - Ducl);

% Comp sensitivity at control T_u
Tu = ss(Aucl, Bucl, Cucl, Ducl);



%%
% *************************************************************************
% *************************************************************************
%
% PACK OUTPUT DATA
%
% *************************************************************************
% *************************************************************************

% Q, R
lq_data.Q = Q;
lq_data.R = R;

% ***********************
%       
% AUGMENTED PLANT STATE SPACE
%  

lq_data.Aaug = Aaug;
lq_data.Baug = Baug;
lq_data.Cr = Cr;


% ***********************
%       
% LQR DESIGN
%  

lq_data.P = P;

lq_data.K = K;
lq_data.Kz = Kz;
lq_data.Ky = Ky;
lq_data.Kr = Kr;


% ***********************
%       
% LOOP BROKEN AT ERROR L_e
%  

lq_data.Ae = Ae;
lq_data.Be = Be;
lq_data.Ce = Ce;
lq_data.De = De;
lq_data.Le = Le;



% ***********************
%       
% LOOP BROKEN AT CONTROLS L_u
%  

lq_data.Au = Au;
lq_data.Bu = Bu;
lq_data.Cu = Cu;
lq_data.Du = Du;
lq_data.Lu = Lu;


% ***********************
%       
% CLOSED-LOOP DYNAMICS BROKEN AT ERROR L_e
%

lq_data.Aecl = Aecl;
lq_data.Becl = Becl;
lq_data.Cecl = Cecl;
lq_data.Decl = Decl;
lq_data.Se = Se;
lq_data.Te = Te;


% ***********************
%       
% CLOSED-LOOP DYNAMICS BROKEN AT CONTROL L_u
%

lq_data.Aucl = Aucl;
lq_data.Bucl = Bucl;
lq_data.Cucl = Cucl;
lq_data.Ducl = Ducl;
lq_data.Su = Su;
lq_data.Tu = Tu;



%%
% *************************************************************************
% *************************************************************************
%
% SAVE DATA
%
% *************************************************************************
% *************************************************************************


if savedata

    % Make directory to save data to
    mkdir(relpath_data)

    % Save data 
    varname = 'lq_data';
    save([relpath_data filename], varname);

end

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


