function hsv_wang_stengel_2000_lq_servo_inout_init(alg_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% HSV PITCH-AXIS MODEL -- LQ-SERVO INNER-OUTER LOOP DESIGN
%
% Brent Wallace  
%
% 2022-09-28
%
% This program performs LQR design on a linearization of a nonlinear
% pitch-axis HSV model. Data is from:
%
%   Q. Wang and R. F. Stengel. "Robust nonlinear control of a hypersonic
%   aircraft." AIAA J. Guid., Contr., & Dyn., 23(4):577–585, July 2000
%
% And:
%
%   C. I. Marrison and R. F. Stengel. "Design of robust control systems for
%   a hypersonic aircraft." AIAA J. Guid., Contr., & Dyn., 21(1):58–63,
%   Jan. 1998.
%
% NOTE: Make sure the MATLAB path is in the folder holding main.m before
% running this program.
%       
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
% LOAD NONLINEAR MODEL
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% System tag
sys = alg_settings.sys;


% *************************************************************************
% 
% UNPACK SETTINGS/PARAMETERS
% 
% *************************************************************************

% System
n = sys.n;                          % System order
m = sys.m;                          % System input dimension
% model = sys.model;                  % System model

% Data saving settings
savedata = isfield(alg_settings, 'relpath_data');

% If data is to be saved, extract the relative path and file name to save
% to
if savedata
    relpath_data = alg_settings.relpath_data;
    filename = alg_settings.filename;
end

% Get Q, R
QV = alg_settings.QV;
RV = alg_settings.RV;
Qg = alg_settings.Qg;
Rg = alg_settings.Rg;

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% Design model
model_d = alg_settings.model_d;


% ***********************
%
% LINEARIZATION TERMS
%

% Linearization params
lin = model_d.lin;
io = lin.io;

% Scaled linear dynamics
Ad = io.Ad;
Bd = io.Bd;
Cd = io.Cd;
Dd = io.Dd;

% \delta_{T} -> V subsystem
AdTV = io.AdTV;
BdTV = io.BdTV;
CdTV = io.CdTV;
DdTV = io.DdTV;
PdTV = io.PdTV;

% \delta_{T} -> V subsystem -- with exact A_{11}
Ad11 = io.Ad11;
Bd11 = io.Bd11;
Cd11 = io.Cd11;
Dd11 = io.Dd11;
Pd11 = io.Pd11;

% \delta_{E} -> \gamma subsystem
AdEg = io.AdEg;
BdEg = io.BdEg;
CdEg = io.CdEg;
DdEg = io.DdEg;
PdEg = io.PdEg;

% \delta_{E} -> \gamma subsystem -- with exact A_{22}
Ad22 = io.Ad22;
Bd22 = io.Bd22;
Cd22 = io.Cd22;
Dd22 = io.Dd22;
Pd22 = io.Pd22;


%%
% *************************************************************************
% *************************************************************************
%
% DESIGN -- VELOCITY SUBSYSTEM
%
% *************************************************************************
% *************************************************************************

% Coordinate transformations
alg_settings.su = 1;
alg_settings.sx = 1;
alg_settings.sy = 1;

% Plant state-space
alg_settings.Ap = AdTV;
alg_settings.Bp = BdTV;
alg_settings.Cp = CdTV;
alg_settings.Dp = DdTV;

% Q, R
alg_settings.Q = QV;
alg_settings.R = RV;

% Perform design
lq_data_V = lq_servo_init(alg_settings);

%%
% *************************************************************************
% *************************************************************************
%
% DESIGN -- VELOCITY SUBSYSTEM -- EXACT A_{11}
%
% *************************************************************************
% *************************************************************************

% Plant state-space
alg_settings.Ap = Ad11;
alg_settings.Bp = Bd11;
alg_settings.Cp = Cd11;
alg_settings.Dp = Dd11;

% Perform design
lq_data_11 = lq_servo_init(alg_settings);


%%
% *************************************************************************
% *************************************************************************
%
% DESIGN -- FPA SUBSYSTEM
%
% *************************************************************************
% *************************************************************************

% Coordinate transformations
alg_settings.su = 1;
alg_settings.sx = eye(3);
alg_settings.sy = 1;

% Plant state-space
alg_settings.Ap = AdEg;
alg_settings.Bp = BdEg;
alg_settings.Cp = CdEg;
alg_settings.Dp = DdEg;

% Q, R
alg_settings.Q = Qg;
alg_settings.R = Rg;

% Perform design
lq_data_g = lq_servo_init(alg_settings);

%%
% *************************************************************************
% *************************************************************************
%
% DESIGN -- FPA SUBSYSTEM -- EXACT A_{22}
%
% *************************************************************************
% *************************************************************************

% Plant state-space
alg_settings.Ap = Ad22;
alg_settings.Bp = Bd22;
alg_settings.Cp = Cd22;
alg_settings.Dp = Dd22;

% Perform design
lq_data_22 = lq_servo_init(alg_settings);


%%
% *************************************************************************
% *************************************************************************
%
% DESIGN -- OVERALL SYSTEM
%
% *************************************************************************
% *************************************************************************

% Coordinate transformations
alg_settings.su = eye(2);
alg_settings.sx = eye(4);
alg_settings.sy = eye(2);

% Plant state-space
alg_settings.Ap = Ad;
alg_settings.Bp = Bd;
alg_settings.Cp = Cd;
alg_settings.Dp = Dd;

% EQUIVALENT "Q", "R" MATRICES

% Q -- Without h
Q = diag([QV(1,1) Qg(1,1) QV(2,2) Qg(2,2) Qg(3,3) Qg(4,4)]);

% Q -- With h
Qh = [  Q           zeros(6,1)
        zeros(1,6)  0           ];

% R
R = diag([RV Rg]);

alg_settings.Q = Q;
alg_settings.R = R;


% Perform design
lq_data_c = lq_servo_init(alg_settings);

% Extract controller
Kc = lq_data_c.K;


%%
% *************************************************************************
% *************************************************************************
%
% FORM COMPOSITE GAIN MATRIX
%
%   \delta_{T} = K_{z,V} * z_V + K_{y,V} e_{V}
%   \delta_{E} = K_{z,\gamma} * z_\gamma + K_{y,\gamma} e_{\gamma} 
%                   - K_{r} x_r
%
% Where x_r = [\theta, q]^T
%
% Inputs: x = [ z      
%               y
%               x_r  ]
%           = [ z_V
%               z_\gamma
%               ---------
%               V
%               \gamma
%               ---------
%               \theta
%               q           ]
%
% Outputs: u = [ \delta_{T}
%                \delta_{E} ]   
%
%
% *************************************************************************
% *************************************************************************



% *************************************************************************
%
% APPROXIMATE A_{jj}
%
% *************************************************************************

% Extract V controller
KV = lq_data_V.K;
KzV = lq_data_V.Kz;
KyV = lq_data_V.Ky;

% Extract \gamma controller
Kg = lq_data_g.K;
Kzg = lq_data_g.Kz;
Kyg = lq_data_g.Ky;
Krg = lq_data_g.Kr;

% Form composite controller -- without h
K = [   KzV     0       KyV     0       zeros(1,2)  
        0       Kzg     0       Kyg     Krg         ];
Kz = K(:,1:m);
Ky = K(:,m+1:2*m);
Kr = K(:,2*m+1:m+n-1);

% Form composite controller -- with h
Kh = [K  zeros(m,1)];
Kzh = Kz;
Kyh = Ky;
Krh = [Kr  zeros(m,1)];


% *************************************************************************
%
% EXACT A_{jj}
%
% *************************************************************************

% Extract V controller
Kz11 = lq_data_11.Kz;
Ky11 = lq_data_11.Ky;

% Extract \gamma controller
Kz22 = lq_data_22.Kz;
Ky22 = lq_data_22.Ky;
Kr22 = lq_data_22.Kr;

% Form composite controller
Kjj = [   Kz11     0       Ky11     0       zeros(1,2)  
        0       Kz22     0       Ky22     Kr22         ];
Kjjz = Kjj(:,1:m);
Kjjy = Kjj(:,m+1:2*m);
Kjjr = Kjj(:,2*m+1:m+n-1);


%%
% *************************************************************************
% *************************************************************************
%
% FORM COMPOSITE GAIN MATRIX -- IRL STATE PARTITION
%
%   \delta_{T} = K_{z,V} * z_V + K_{y,V} e_{V}
%   \delta_{E} = K_{z,\gamma} * z_\gamma + K_{y,\gamma} e_{\gamma} 
%                   - K_{r} x_r
%
% Where x_r = [\theta, q]^T
%
% Inputs: x = [ x_1
%               x_2     ]
%         x_1  = [  z_V
%                   V           ]
%                   z_\gamma
%         x_2  = [  z_\gamma
%                   \gamma
%                   ---------
%                   \theta
%                   q           ]
%
% Inputs: u = [ \delta_{T}
%               \delta_{E} ]   
%
%
% *************************************************************************
% *************************************************************************

% EQUIVALENT "Q" MATRIX

% Q -- Without h
Qirl = [QV          zeros(2,4)
        zeros(4,2)  Qg          ];

% Q -- With h
Qirlh = [   Qirl        zeros(6,1)
            zeros(1,6)  0           ];

% EQUIVALENT CONTROLLER -- DIAGONAL
Kcirl = [Kc(:,1) Kc(:,3) Kc(:,2) Kc(:,4:6)];

% EQUIVALENT CONTROLLER -- DIAGONAL
Kdirl = [   KV             zeros(1,4)
            zeros(1,2)      Kg          ];


%%
% *************************************************************************
% *************************************************************************
%
% PREFILTER POLE LOCATIONS
%
% A prefilter is inserted before the reference command in each
% channel, of form
%
%   W_i(s) =    a_i
%             --------,         i = 1,...,m
%
%
% *************************************************************************
% *************************************************************************

% Calculate zero locations automatically
gV = KyV;
zV = KzV / gV;
gg = Kyg;
zg = Kzg / gg;
pfavec = [zV; zg];


% Make pre-filter V
AwV = -zV;
BwV = zV;
CwV = 1;
DwV = 0;
WV = ss(AwV,BwV,CwV,DwV);
lq_data_V.W = WV;

% Make pre-filter \gamma
Awg = -zg;
Bwg = zg;
Cwg = 1;
Dwg = 0;
Wg = ss(Awg,Bwg,Cwg,Dwg);
lq_data_g.W = Wg;

% Make pre-filter for total system
Aw = [AwV 0; 0 Awg];
Bw = [BwV 0; 0 Bwg];
Cw = [CwV 0; 0 Cwg];
Dw = [DwV 0; 0 Dwg];
W = ss(Aw,Bw,Cw,Dw);


%%
% *************************************************************************
% *************************************************************************
%
% PACK OUTPUT DATA
%
% *************************************************************************
% *************************************************************************

% Individual loop data
lq_data.lq_data_V = lq_data_V;
lq_data.lq_data_11 = lq_data_11;
lq_data.lq_data_g = lq_data_g;
lq_data.lq_data_22 = lq_data_22;
lq_data.lq_data_tot = lq_data_c;

% Composite Q, R
lq_data.Q = Q;
lq_data.Qh = Qh;
lq_data.R = R;

% Composite controller -- without h
lq_data.K = K;
lq_data.Kz = Kz;
lq_data.Ky = Ky;
lq_data.Kr = Kr;

% Composite controller -- with h
lq_data.Kh = Kh;
lq_data.Kzh = Kzh;
lq_data.Kyh = Kyh;
lq_data.Krh = Krh;

% Composite controller -- exact A_{jj}
lq_data.Kjj = Kjj;
lq_data.Kjjz = Kjjz;
lq_data.Kjjy = Kjjy;
lq_data.Kjjr = Kjjr;

% Composite data -- IRL state partition
lq_data.Qirl = Qirl;
lq_data.Qirlh = Qirlh;
lq_data.Kcirl = Kcirl;
lq_data.Kdirl = Kdirl;

% Prefilter data
lq_data.pfavec = pfavec;
lq_data.W = W;


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


% Display done
disp('************************')
disp('*')
disp(['* LQ SERVO INNER/OUTER DESIGN COMPLETE'])
disp('*')
disp('************************')

% Calculate zeros, gains
gV = lq_data_V.Ky;
zV = lq_data_V.Kz / gV;
gg = lq_data_g.Ky;
zg = lq_data_g.Kz / gg;

gi = lq_data_g.Kr(1);
zi = lq_data_g.Kr(2) / gi;

disp(['g_{i} =  ' num2str(gi)])        
disp(['z_{i} =  ' num2str(zi)])        
disp(['g_{V} =  ' num2str(gV)])        
disp(['z_{V} =  ' num2str(zV)])
disp(['g_{g} =  ' num2str(gg)])
disp(['z_{g} =  ' num2str(zg)])

