function out_data = alg_wang_stengel_2000_lq_servo_inout(alg_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% LQ SERVO INNER-OUTER CLOSED-LOOP CONTROL
%
% Brent Wallace  
%
% 2022-08-13
%
% This program, given a system, reference command, and LQ servo inner/outer
% loop controller, simulates the resulting closed-loop response.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings  struct with the following fields:
%   
%   sys                     (Struct) Contains system params.
%   K                       (m x (m+n) Matrix) LQ servo inner/outer
%                           controller
%   r_sett                  (Struct) Contains reference command settings.
%                           See eval_xr.m for details.
%   tsim                    (Scalar) Time t > 0 to simulate closed-loop
%                           response for.
%   model_nom_tag           (String) Tag for nominal model. Can be
%                           'default' for default model (\nu = 1) or
%                           'perturbed' for perturbed model (\nu ~= 1).
%   model_sim_tag           (String) Tag for simulation model. Can be
%                           'default' for default model (\nu = 1) or
%                           'perturbed' for perturbed model (\nu ~= 1).
%   lin1nonlin0             (Bool) Model linear (=1) or nonlinear (=0).
%   c1dc0                   (Bool) Needed for linear model (lin1nonlin0 =
%                           1) only. Whether to simulate entire HSV model
%                           (i.e., with altitude h included in the state)
%                           or model without h.
%   Pmp1nmp0                (Bool) Plant model MP (=1) or NMP (=0).
%   ndi1notndi0             (Bool) Algorithm is NDI (=1) or not NDI (=0).
%   pf1nopf0                (Bool) Do prefilter (=1) or not (=0).
%   pfavec                  (m-dim vector, OPTIONAL) If prefilter is used,
%                           contains the prefilter pole locations. Declare
%                           only if pf1nopf0 = 1.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%   tvec                    ('simlength'-dimensional vector) vector of time
%                           indices corresponding to the simulation time
%                           instants over the course of the algorithm
%                           execution.
%   xmat                    ('simlength' x n matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%   umat                    ('simlength' x m matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose m-columns are the control signal u(t)
%                           at the respective time instant.
%   inds                    (Struct) Indices of how state vector is
%                           partitioned for the ode45 simulation.
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;

% If model linear, see if desired to simulate on the coupled plant (=1) or
% decoupled plant
if lin1nonlin0
    c1dc0 = alg_settings.c1dc0;
end


% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% 
% GLOBAL VARIABLES
% 
% *************************************************************************

global sys;


% Reference signal r(t) settings
global r_sett;

% Do nominal min-phase model (=1) or nonmin-phase model (=0)
global u_sett;

        

% *************************************************************************
% 
% UNPACK ALGORITHM SETTINGS/PARAMETERS
% 
% *************************************************************************

% System
sys = alg_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension
model = sys.model;                  % System model

% Reference signal r(t) settings
r_sett = alg_settings.r_sett;

% Simulation length
tsim = alg_settings.tsim;

% ***********************
%       
% TRIM CONDITIONS
%
% x_v = [V, \gamma, h, \alpha, q, \delta_{T}, \dot{\delta}_{T}] in R^{7}
%    

% Numerically solved-for values -- WITH ELEVATOR-LIFT EFFECTS
xe_dE = model.trimconds.xe_dE;
ue_dE = model.trimconds.ue_dE;


% Nominal model
model_nom_tag = alg_settings.model_nom_tag;
switch model_nom_tag
    case 'default'
        model_nom = sys.model;
    case 'perturbed'
        model_nom = sys.model_nu;
end
xe_nom = model_nom.trimconds.xe_dE;
ue_nom = model_nom.trimconds.ue_dE;

% Simulation model
model_sim_tag = alg_settings.model_sim_tag;
switch model_sim_tag
    case 'default'
        model_sim = sys.model;
    case 'perturbed'
        model_sim = sys.model_nu;
end
xe_sim = model_sim.trimconds.xe_dE;
ue_sim = model_sim.trimconds.ue_dE;


% ***********************
%       
% PRESET SETTINGS
%   

% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;

% If model linear, see if desired to simulate on the coupled plant (=1) or
% decoupled plant
if lin1nonlin0
    c1dc0 = alg_settings.c1dc0;
end

% Decide whether to include the height mode in sim (=1) or not (=0)
if lin1nonlin0
    has_h = c1dc0;
else
    has_h = 1;
end

% Plant model MP (=1) or NMP (=0)
Pmp1nmp0 = alg_settings.Pmp1nmp0;

% Algorithm is NDI (=1) or not NDI (=0)
ndi1notndi0 = alg_settings.ndi1notndi0;

% Do prefilter (=1) or not (=0)
pf1nopf0 = alg_settings.pf1nopf0;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    pfavec = alg_settings.pfavec;   
end


% ***********************
%       
% CONTROLLER
%

% Controller
K = alg_settings.K;

% DEBUGGING: Print done loading
disp('***** LOADING PARAMETERS COMPLETE *****')

% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************

% ***********************
%       
% MISCELLANEOUS VARIABLES
%

% Initial condition
x0 = alg_settings.x0;  

% Take out height mode if required
if ~has_h
    x0_sim = x0([1:model.indh-1 model.indh+1:n]);
else
    x0_sim = x0;
end

% Append integrator ICs
x0_sim = [  x0_sim
            zeros(m,1)  ];

% If prefilter is used, append ICs for the prefilter states
if pf1nopf0
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end

% ***********************
%       
% CONTROL SETTINGS
%   

% Store nominal model in global settings
u_sett.model_nom = model_nom;
u_sett.xe_nom = xe_nom;
u_sett.ue_nom = ue_nom;

% Store simulation model in global settings
u_sett.model_sim = model_sim;
u_sett.xe_sim = xe_sim;
u_sett.ue_sim = ue_sim;

% Model linear (=1) or nonlinear (=0)
u_sett.lin1nonlin0 = lin1nonlin0;

% Plant model MP (=1) or NMP (=0)
u_sett.Pmp1nmp0 = Pmp1nmp0;

% If model linear, see if desired to simulate on the coupled plant (=1) or
% decoupled plant
if lin1nonlin0
    u_sett.c1dc0 = c1dc0;
end

% Whether to include the height mode in sim (=1) or not (=0)
u_sett.has_h = has_h;

% Algorithm is NDI (=1) or not NDI (=0)
u_sett.ndi1notndi0 = ndi1notndi0;

% Do prefilter (=1) or not (=0)
u_sett.pf1nopf0 = pf1nopf0;

% Trim
u_sett.xe = xe_dE;
u_sett.ue = ue_dE;

% Indices of state variables to be tracked
inds_xr = alg_settings.inds_xr;
u_sett.inds_xr = inds_xr;

% Number of states in simulated plant
if ~has_h
    nd = n - 1;
else
    nd = n;
end
u_sett.nd = nd;

% Coordinate transformations
u_sett.su = model.lin.io.sud;
u_sett.sx = model.lin.io.sxd;
u_sett.sy = model.lin.io.syd;

% Controller
u_sett.K = K;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    u_sett.pfavec = pfavec;   
end


% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Time vector, state trajectory, control signal
tvec = [];
xmat = [];
umat = [];


% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************



% *************************************************************************
% *************************************************************************
%
% RUN SIMULATION WITH FINAL CONTROLLER
% 
% *************************************************************************
% *************************************************************************


% ***********************
%       
% RUN SIMULATION
%

% Time span for simulation
tspan = [0, tsim];

% Run simulation
[t, x] = ode45(@odefunct, tspan, x0_sim);

% % DEBUGGING: Plot states while calling ode45
% options = odeset('OutputFcn',@odeplot);
% figure(100);
% [t, x] = ode45(@odefunct, tspan, x0_sim, options); 


% ***********************
%       
% STORE DATA
%

% Store time data
tvec = [    tvec
            t       ];


% Store system state data
xmat = [    xmat
            x       ];


% DEBUGGING: Print done simulating
disp('***** SIMULATION COMPLETE *****')



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PREPARE OUTPUT DATA
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% CONTROL SIGNAL
% 
% *************************************************************************

% Initialize empty matrix
umat = zeros(size(tvec,1), m);

% Calculate control
for k = 1:size(tvec,1)
    
    % Get time
    t = tvec(k);

    % Get state vector 
    xs = xmat(k,:)';
    
    % Evaluate control 
    u = uxt_alg(xs, t);
                
    % Store control
    umat(k,:) = u';

end

% Store control signal
out_data.umat = umat;

% *************************************************************************
%
% TIME, STATE
% 
% *************************************************************************

% Time vector
out_data.tvec = tvec;

% Rescale integrator statees to reflect original units
xmat(:,nd+1:nd+m) = (inv(u_sett.sy) * xmat(:,nd+1:nd+m)')';

% If a prefilter was used, rescale and shifted filtered reference command
% responses to reflect trim (pre-transformation)
if pf1nopf0
   xmat(:,nd+m+1:nd+m+m) = (inv(u_sett.sy) * xmat(:,nd+m+1:nd+m+m)')' ...
       +  [xe_sim(inds_xr(1)); xe_sim(inds_xr(2))]';
   out_data.xmat = xmat;
end

% If height mode not present, then add the constant equilibrium height
% h(t) = h_{e} into the state
if ~has_h
    xmat = [xmat(:,1:model.indh-1) ...
                xe_sim(model.indh)*ones(size(tvec,1),1) ...
                xmat(:,model.indh:end)];
end

% Store modified state vector
out_data.xmat = xmat;

% Indices of how state vector is partitioned
inds.indsx = (1:n)';
inds.indsz = (n+1:n+m)';
if pf1nopf0
    inds.indspf = (n+m+1:n+m+m)';
end
out_data.inds = inds;

% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE DYNAMICS 
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function xdot = odefunct(t, x)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Control settings
global r_sett;
global u_sett;

% Extract simulation model
model_sim = u_sett.model_sim;

% Get number of states in simulation plant
nd = u_sett.nd;

% Get coordinate transformations
% sx = u_sett.sx;
% su = u_sett.su;
sy = u_sett.sy;

% Extract plant states
xp = x(1:nd);

% % Get equilibrium point x_e (pre-transformation)
% xe = u_sett.xe;
% 
% % Get equilibrium control u_e (pre-transformation)
% ue = u_sett.ue;

% % Get equilibrium point x_e (pre-transformation) -- simulated system
% xe = u_sett.xe_sim;
% 
% % Get equilibrium control u_e (pre-transformation) -- simulated system
% ue = u_sett.ue_sim;

% Get equilibrium point x_e (pre-transformation) -- nominal system
xe = u_sett.xe_nom;

% Get equilibrium control u_e (pre-transformation) -- nominal system
ue = u_sett.ue_nom;

% If system height mode not used, take out the height mode from equilibrium
if ~u_sett.has_h
    xe = xe([1:sys.model.indh-1 sys.model.indh+1:n]);
end

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Evaluate drift dynamics
if u_sett.lin1nonlin0
    % System linear
    if u_sett.c1dc0
        f_x = model_sim.lin.ApdE * tx;
    else
        f_x = model_sim.lin.io.Apnohio * tx;
    end
else
    % System nonlinear
    f_x = model_sim.fx(xp);
end

% Evaluate input gain matrix
if u_sett.lin1nonlin0
    % System linear
    if u_sett.c1dc0
        g_x = model_sim.lin.BpdE;
    else
        g_x = model_sim.lin.io.Bpnohio;
    end
else
    % System nonlinear
    if u_sett.Pmp1nmp0
        g_x = model_sim.gx(xp);
    else
        g_x = model_sim.gxdE(xp);
    end
end

% Calculate control signal
u = uxt_alg(x, t);
% u = [0.183; -0.0066];

% If model is linear, apply \tilde{u} = u - u_{e} (pre-transformation)
if u_sett.lin1nonlin0
    u = u - ue;
end

% % Calculate state derivative
% xdot = [    f_x + g_x * u ];

% Evaluate reference trajectory r(t) (pre-transformation)
rt = eval_xr(t, r_sett);

% Evaluate reference trajectory r(t) (post-transformation)
yr = [rt(1); rt(5)];
yrp = sy * yr;

% % Get y_{r}(t) (post-transformation)
% yrp = [rtp(1); rtp(5)];

% Get y(t) (post-transformation)
yp = sy * x(u_sett.inds_xr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(u_sett.inds_xr);

% Get (small-signal) reference command r (post-transformation)
trp = yrp - yep;

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% If prefilters used, get filtered reference command (post-transformation)
if u_sett.pf1nopf0
    trfp = x(nd+m+1:nd+m+m);
end

% Evaluate integrator state derivative \dot{z} (post-transformation)
if u_sett.pf1nopf0
    zdot = -(trfp - typ);
else
    zdot = -(trp - typ);
end

% Append integral augmentation state derivatives if executing LQ servo
xdot = [    f_x + g_x * u
            zdot            ];

% If prefilter inserted, evaluate prefilter dynamics (post-transformation)
if u_sett.pf1nopf0
    pfdot = -diag(u_sett.pfavec) * trfp + diag(u_sett.pfavec) * trp;
    xdot = [    xdot
                pfdot   ];
end        



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE CONTROL SIGNAL u(x, t)
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function u = uxt_alg(x, t)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Control settings
global r_sett;
global u_sett;

% Get number of states in simulation plant
nd = u_sett.nd;

% Get coordinate transformations
sx = u_sett.sx;
su = u_sett.su;
sy = u_sett.sy;

% ***********************
%       
% LQ REFERENCE COMMAND FOLLOWING
%   

% Get equilibrium point x_e (pre-transformation) -- nominal system
xe = u_sett.xe_nom;

% Get equilibrium control u_e (pre-transformation) -- nominal system
ue = u_sett.ue_nom;

% Take out height mode
xe = xe([1:sys.model.indh-1 sys.model.indh+1:n]);

% Evaluate reference trajectory r(t) (pre-transformation)
rt = eval_xr(t, r_sett);

% Evaluate reference trajectory r(t) (post-transformation)
yr = [rt(1); rt(5)];
yrp = sy * yr;

% % Get y_{r}(t) (post-transformation)
% yrp = [rtp(1); rtp(5)];

% % Get y(t) (post-transformation)
% yp = sy * xs(u_sett.inds_xr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(u_sett.inds_xr);

% Get (small-signal) reference command r (post-transformation)
trp = yrp - yep;

% % Get (small-signal) output y (post-transformation)
% typ = yp - yep;

% If prefilters used, get filtered reference command (post-transformation)
if u_sett.pf1nopf0
    trfp = x(nd+m+1:nd+m+m);
    trp = trfp;
end

% Get contoller
K = u_sett.K;

% Extract parts of feedback gain matrix K corresponding to each of
% the states in the partition
Kz = K(:,1:m);
Ky = K(:,m+1:2*m);
Kr = K(:,2*m+1:m+n-1);

% Extract the plant states x_p (pre-transformation)
xp = x(1:nd);

% If system has height mode, take out the height mode
if u_sett.has_h
    xp = xp([1:sys.model.indh-1 sys.model.indh+1:n]);
end

% Extract the integral augmentation states z (post-transformation)
z = x(nd+1:nd+m);

% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Apply coordinate transformation
% x' = S_x x
%    = [    y
%           x_r     ]
txp = sx * tx;
typ = txp(1:m);
txr = txp(m+1:n-1);

% Calculate (small-signal) tracking error e = r - y (post-transformation)
e = trp - typ;

% Calculate control (post-transformation) 
tup = Kz * (-z) + Ky * e - Kr * txr;

% Calculate linear control (pre-tranformation)
tu = inv(su) * tup;

% Calculate final control u = u_e + \tilde{u}
u = ue + tu;





