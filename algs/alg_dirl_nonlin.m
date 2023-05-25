function out_data = alg_dirl_nonlin(alg_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% DECENTRALIZED EXCITABLE INTEGRAL REINFORCEMENT LEARNING (dEIRL) ALGORITHM
%
% Brent Wallace  
%
% 2022-11-14
%
% This program implements the dEIRL algorithm developed in
%
% B. A. Wallace, J. Si, "Physics-Based Integral Reinforcement Learning: New
% Control Design Algorithms with Theoretical Insights and Performance
% Guarantees" TNNLS 2023.
%
% For control of hypersonic vehicles.
%
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings  struct with the following fields:
%   
%   sys                     (Struct) contains system tag/model info. 
%   Ts                      (Double) Base sample period to collect data (s)
%   loop_cell               (N-entry Struct) This cell array contains the
%                           learning hyperparameters in each of the 1 <= j
%                           <= N loops of the system 'sys'. Each entry 1 <=
%                           j <= N has the following fields (see
%                           config_preset.m):
%       indsx               (n_j-dim Vector -- NOT HYPERPARAM) Contains the 
%                           indices of the total state vector x
%                           corresponding to the states in this loop x_j
%                           \in R^{n_j}.
%       indsy               (m_j-dim Vector -- NOT HYPERPARAM) Contains the
%                           indices of the total state vector x
%                           corresponding to the outputs y_j \in R^{m_j} in
%                           this loop.
%       Q                   (n_j x n_j Matrix) State penalty 
%                           Q_j = Q_j^T >= 0 in this loop j.
%       R                   (m_j x m_j Matrix) Control penalty
%                           R_j = R_j^T > 0 in this loop j.
%       K0                  (m_j x n_j Matrix) Initial stabilizing
%                           controller K_{0,j} in this loop j. NOTE: This
%                           is configurable, but it should be set by the
%                           LQR design performed in the respective system's
%                           controller initialization function (see
%                           /config/config_controllers_<system>.m).
%       nTs                 (Integer) Multiple of the base sample period Ts
%                           to sample learning data at in this loop j.
%       l                   (Integer) Number of data samples to collect for
%                           this loop j.
%       istar               (Integer) Number of learning iterations to
%                           execute for this loop j.
%   r_sett_train            (Struct) Contains parameters for the
%                           reference excitation r(t) \in R^m.
%                           This basically allows a sum of sinusoids to be
%                           injected at each input, and the fields
%                           correspond to the desired amplitude, frequency,
%                           and phase of the sinusoids. For further
%                           description, see eval_xr.m. In what follows,
%                           suppose M sinusoidal terms are injected in each
%                           input channel (this M is configurable). Then
%                           'r_sett_train' has the following fields:
%       cos1_sin0           (m x M Matrix) In input channel 1 <= i <= m,
%                           make the sinusoid 1 <= j <= M a cosine (=1) or
%                           sine (=0).
%       Amat                (m x M Matrix) The (i,j)th entry of this matrix
%                           determines the amplitude of the sinusoid 
%                           1 <= j <= M injected in input channel 
%                           1 <= i <= m.
%       Tmat                (m x M Matrix) The (i,j)th entry of this matrix
%                           determines the period of the sinusoid 
%                           1 <= j <= M injected in input channel 
%                           1 <= i <= m.
%   noise                   (Struct) Contains parameters for the plant
%                           input excitation d(t) \in R^m. Fields are
%                           analogous to r_sett_train.
%
% ***
%   wnom1sim0               (Bool -- FOR DEBUGGING ONLY) Trim point for the 
%                           nominal linearization A used in the
%                           nonlinearity w = f(x) - A x. At nominal trim
%                           (=1) or simulation (i.e., perturbed system)
%                           trim (=0). Note: For systems
%                           in which the trim point is invariant with
%                           respect to modeling error, this parameter is
%                           redundant. We developed this parameter for
%                           algorithm debugging purposes, all studies use
%                           the nominal system equilibrium point.
%       
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%   dirl_data               (Struct) Contains algorithm learning output
%                           data. Has the following fields:
%       P_cell              (N-dim Cell) The j-th entry contains a matrix
%                           with all the ALE solutions P_{i,j} in loop j.
%       K_cell              (N-dim Cell) The j-th entry contains a matrix
%                           with all the controllers K_{i,j} in loop j.
%       lq_data             (Struct) Contains the LQR data for the FINAL
%                           controllers in each loop. This struct is used
%                           by other programs to pull final controller data
%                           for plotting, simulation, etc.
%       cmat_cell           (N-dim cell) The j-th entry contains the 
%                           VECTORIZED ALE solutions v(P_{i,j}) in loop j
%                           (used for plotting weight responses).
%       cond_A_vec_cell     (N-dim cell) The j-th entry contains an
%                           i_{j}^{*}-dim vector, the i-th entry of which
%                           is the conditioning of the least-squares matrix
%                           A_{i,j} in loop j at iteration i.
%   tlvec                   ('simlength'-dimensional vector) vector of time
%                           indices corresponding to the simulation time
%                           instants over the course of the learning
%                           algorithm simulation (if executed).
%   xlmat                   ('simlength' x n matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%   ulmat                   ('simlength' x m matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose m-columns are the control signal u(t)
%                           at the respective time instant.
%   tvec, xmat, umat        (Matrices) Analogous to tlvec, xlmat, ulmat,
%                           but for the POST-LEARNING simulation conducted
%                           on the final controller (if executed).
%   inds                    (Struct) Contains the state vector partition
%                           for the numerical simulation conducted by
%                           ode45. E.g., to find where the plant states are
%                           in the ode45 state vector, query inds.indsx.
%   loopdata                (N-dim Struct) Contains system data for each of
%                           the N loops (e.g., state dimension n_j, etc.)
%                           This is mainly used for back-end purposes.      
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


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

% Control settings
global u_sett;



% *************************************************************************
% 
% UNPACK ALGORITHM SETTINGS/PARAMETERS
% 
% *************************************************************************

% DEBUGGING: Display state trajectory plots during learning
dispxl = 0;

% System
sys = alg_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension
model = sys.model;                  % System model
model_nu = sys.model_nu;            % System model -- perturbed

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% Do learning (=1) or not (=0)
dolearning = alg_settings.dolearning;

% Do post-learning sim (=1) or not (=0)
dopostlearning = alg_settings.dopostlearning; 

% Relative path to load learning data from if learning isn't active
relpath_data = alg_settings.relpath_data;
filename_data = alg_settings.filename_data;

% Reference signal r(t) settings -- training phase
r_sett_train = alg_settings.r_sett_train;

% Reference signal r(t) settings
r_sett = alg_settings.r_sett;
r_sett.r_sett_train = r_sett_train;

% Exploration noise
noise = alg_settings.noise;   

% Base sample period (sec)
Ts = alg_settings.Ts;

% Post-learning simulation length
tsim = alg_settings.tsim;

% Do x_3 loop (=1) or not (=0)
dox3 = alg_settings.dox3;

% Overwrite current controller (=1) or not (=0)
updatecontroller = alg_settings.updatecontroller;

% If desired to update controller, get the file path
if updatecontroller
    relpath_data_dirl = alg_settings.relpath_data;
    filename_data_dirl = alg_settings.filename_data;
end

% Use the nominal linearization A for w = f(x) - A x at nominal trim (=1)
% or simulation trim (=0)
wnom1sim0 = alg_settings.wnom1sim0;

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

% Optimal LQ data
lq_data_opt = alg_settings.lq_data_opt;
lq_data_opt_nu = alg_settings.lq_data_opt_nu;

% Optimal LQ data -- simulation model
switch model_sim_tag
    case 'default'
        lq_data_opt_sim = lq_data_opt;
    case 'perturbed'
        lq_data_opt_sim = lq_data_opt_nu;
end

% ***********************
%       
% TRIM CONDITIONS
%

% Numerically solved-for values
xe = model.trimconds.xe_dE;
ue = model.trimconds.ue_dE;


% ***********************
%       
% LOOP DATA
%  

% Holds settings for each loop
loop_cell = alg_settings.loop_cell;

% ***********************
%       
% PRESET SETTINGS
%   

% Model linear (=1) or nonlinear (=0)
lin1nonlin0 = alg_settings.lin1nonlin0;

% Plant model MP (=1) or NMP (=0)
Pmp1nmp0 = alg_settings.Pmp1nmp0;

% Do prefilter (=1) or not (=0)
pf1nopf0 = alg_settings.pf1nopf0;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    pfavec = alg_settings.pfavec;   
end



% DEBUGGING: Print done loading
disp('***** LOADING PARAMETERS COMPLETE *****')

% *************************************************************************
% 
% ALGORITHM INITIALIZATION
% 
% *************************************************************************



% ***********************
%       
% CONTROL SETTINGS
%   

% Exploration noise
u_sett.noise = noise;

% Learning flag
u_sett.islearning = 1;

% Model linear (=1) or nonlinear (=0)
u_sett.lin1nonlin0 = lin1nonlin0;

% Plant model MP (=1) or NMP (=0)
u_sett.Pmp1nmp0 = Pmp1nmp0;

% Do prefilter (=1) or not (=0)
u_sett.pf1nopf0 = pf1nopf0;

% Trim
u_sett.xe = xe;
u_sett.ue = ue;

% Coordinate transformations
u_sett.su = model.lin.io.sud;
u_sett.sx = model.lin.io.sxd;
u_sett.sxh = model.lin.io.sxdh;
u_sett.sy = model.lin.io.syd;

% If prefilter is used, save the prefilter pole locations
if pf1nopf0
    u_sett.pfavec = pfavec;   
end

% Do x_3 loop (=1) or not (=0)
u_sett.dox3 = dox3;

% Use the nominal linearization A for w = f(x) - A x at nominal trim (=1)
% or simulation trim (=0)
u_sett.wnom1sim0 = wnom1sim0;

% ***********************
%       
% LOOP DATA
%  

% Get number of loops
numloops = size(loop_cell,1);
loopdata.numloops = numloops;

% Store state, output dimensions in each loop
nxztot = 0;
nxzbtot = 0;
for k = 1:numloops
    loop_cell{k}.nx = size(loop_cell{k}.indsx, 1);
    loop_cell{k}.ny = size(loop_cell{k}.indsy, 1);
    loop_cell{k}.nxz = loop_cell{k}.nx + loop_cell{k}.ny;
    loop_cell{k}.nxzb = loop_cell{k}.nxz * (loop_cell{k}.nxz + 1) / 2;
    nxztot = nxztot + loop_cell{k}.nxz;
    nxzbtot = nxzbtot + loop_cell{k}.nxzb;
end
if dox3
    nxztot = nxztot + 1;
end
nxztotb = nxztot * (nxztot + 1) / 2;
loopdata.nxztotb = nxztotb;
loopdata.nxztot = nxztot;

% Whether or not to perform learning in each loop
doloopvec = alg_settings.doloopvec;
doloopvec = logical(doloopvec);
loopdata.doloopvec = doloopvec;

% Number of iterations to conduct for each loop
istarvec = zeros(numloops,1);
for k = 1:numloops
    istarvec(k) = loop_cell{k}.istar;
end
loopdata.istarvec = istarvec;

% Number of samples to collect per iteration for each loop
lvec = zeros(numloops,1);
for k = 1:numloops
    lvec(k) = loop_cell{k}.l;
end
loopdata.lvec = lvec;

% Integer multiples of base sample period to collect data at for each loop
nTsvec = zeros(numloops,1);
for k = 1:numloops
    nTsvec(k) = loop_cell{k}.nTs;
end
loopdata.nTsvec = nTsvec;

% Determine total number of samples to collect for each loop
nsampvec = zeros(numloops,1);
for k = 1:numloops
    nsampvec(k) = lvec(k) * nTsvec(k);
end
loopdata.nsampvec = nsampvec;

% Max total number of samples to collect during learning
% NOTE: Only consider active loops
maxnsamp = max(nsampvec);
loopdata.maxnsamp = maxnsamp;

% Max i^{*} among the active loops
maxistar = max(istarvec .* doloopvec);
loopdata.maxistar = maxistar;


% ***********************
%       
% LOOP DATA -- MASTER
%  

% Number of loops
numloops_m = m;

% Declare empty array
loop_cell_m = cell(numloops_m,1);

% Indices of state vector in each loop
loop_cell_m{1}.indsx = 1;
loop_cell_m{1}.indsy = 1;
loop_cell_m{2}.indsx = (2:4)';
loop_cell_m{2}.indsy = 2;

% Store state, output dimensions in each loop
for k = 1:numloops_m
    loop_cell_m{k}.nx = size(loop_cell_m{k}.indsx, 1);
    loop_cell_m{k}.ny = size(loop_cell_m{k}.indsy, 1);
    loop_cell_m{k}.nxz = loop_cell_m{k}.nx + loop_cell_m{k}.ny;
    loop_cell_m{k}.nxzb = loop_cell_m{k}.nxz*(loop_cell_m{k}.nxz + 1) / 2;
end


% ***********************
%       
% STATE VECTOR PARTITION INDICES
%   

% Indices of how state vector is partitioned
indsxr = alg_settings.inds_xr;

% Vehicle states
cnt = 1;
len = n;
indsx = (cnt:cnt+len-1)';
cnt = cnt + len;

% Integral augmentation states
len = m;
indsz = (cnt:cnt+len-1)';
cnt = cnt + len;

% Pre-filter states
if pf1nopf0
    len = m;
    indspf = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Kronecker product integrals -- (x, x)
len = nxztotb;
indsIbx = (cnt:cnt+len-1)';
cnt = cnt + len;

% Kronecker product integrals -- (x_k, w_k)
indsIBxw = cell(numloops+1,1);
len = nxzbtot;
indsIBxw{numloops+1} = (cnt:cnt+len-1)';
for k = 1:numloops
    len = loop_cell{k}.nxzb;
    indsIBxw{k} = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Kronecker product integrals -- (x_k, \tilde{w}_k)
indsIBxtw = cell(numloops+1,1);
len = nxzbtot;
indsIBxtw{numloops+1} = (cnt:cnt+len-1)';
for k = 1:numloops
    len = loop_cell{k}.nxzb;
    indsIBxtw{k} = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Kronecker product integrals -- (x_k, g_k(x)u)
indsIBxgu = cell(numloops+1,1);
len = nxzbtot;
indsIBxgu{numloops+1} = (cnt:cnt+len-1)';
for k = 1:numloops
    len = loop_cell{k}.nxzb;
    indsIBxgu{k} = (cnt:cnt+len-1)';
    cnt = cnt + len;
end

% Output integrals \int y
len = m;
indsIy = (cnt:cnt+len-1)';
cnt = cnt + len;

% TOTAL SIMULATION STATE VECTOR LENGTH
n_sim = cnt - 1;
u_sett.n_sim = n_sim;

% Indices of loop-wise Kronecker product integrals
indsIBxwloc = cell(numloops,1);
cntloc = 1;
for k = 1:numloops
    len = loop_cell{k}.nxzb;
    indsIBxwloc{k} = (cntloc:cntloc+len-1)';
    cntloc = cntloc + len;
end

% Indices of loop-wise state vectors in the IRL state partition
cntirl = 1;
if dox3
    indsxirl = cell(numloops+1,1);
else
    indsxirl = cell(numloops,1);
end
for k = 1:numloops
    len = loop_cell{k}.nxz;
    indsxirl{k} = (cntirl:cntirl+len-1)';
    cntirl = cntirl + len;
end
if dox3
    len = 1;
    indsxirl{numloops+1} = (cntirl:cntirl+len-1)';
    cntirl = cntirl + len;
end

% Indices of loop-wise state vectors in the IRL state partition -- master
cntirl = 1;
indsxirl_m = cell(numloops_m,1);
for k = 1:numloops_m
    len = loop_cell_m{k}.nxz;
    indsxirl_m{k} = (cntirl:cntirl+len-1)';
    cntirl = cntirl + len;
end

% Store
inds.indsxr = indsxr;
inds.indsx = indsx;
inds.indsz = indsz;
if pf1nopf0
    inds.indspf = indspf;
end
inds.indsIbx = indsIbx;
inds.indsIBxw = indsIBxw;
inds.indsIBxtw = indsIBxtw;
inds.indsIBxgu = indsIBxgu;
inds.indsIy = indsIy;
inds.indsxirl = indsxirl;
inds.indsxirl_m = indsxirl_m;
u_sett.inds = inds;


% Store loop_cell struct in global settings
u_sett.loop_cell = loop_cell;
u_sett.numloops = numloops;
u_sett.loop_cell_m = loop_cell_m;
u_sett.numloops_m = numloops_m;


% ***********************
%       
% GET STATE-SPACE MATRICES FOR EACH LOOP
%   

% Nominal system
if wnom1sim0
    [A_cell_nom, B_cell_nom] = make_AB_cells(model_nom.lin);
else
    switch model_sim_tag
        case 'default'
            switch model_nom_tag
                case 'default'
                    [A_cell_nom, B_cell_nom] = make_AB_cells(model.lin);
                case 'perturbed'
                    [A_cell_nom, B_cell_nom] = ...
                        make_AB_cells(model_nu.lin_atnom);
            end
        case 'perturbed'
            switch model_nom_tag
                case 'default'
                    [A_cell_nom, B_cell_nom] =...
                        make_AB_cells(model.lin_atnu);
                case 'perturbed'
                    [A_cell_nom, B_cell_nom] = make_AB_cells(model_nu.lin);
            end
    end
end

% Simulation system
[A_cell_sim, B_cell_sim] = make_AB_cells(model_sim.lin);

% Store
model_nom.A_cell = A_cell_nom;
model_nom.B_cell = B_cell_nom;
model_sim.A_cell = A_cell_sim;
model_sim.B_cell = B_cell_sim;

% Store nominal model in global settings
u_sett.model_nom = model_nom;
u_sett.xe_nom = xe_nom;
u_sett.ue_nom = ue_nom;

% Store simulation model in global settings
u_sett.model_sim = model_sim;
u_sett.xe_sim = xe_sim;
u_sett.ue_sim = ue_sim;


% ***********************
%       
% ICS
%

% Initial condition
x0 = alg_settings.x0;  

% Append integrator ICs
x0_sim = [  x0
            zeros(m,1)  ];

% If prefilter is used, append ICs for the prefilter states
if pf1nopf0
    x0_sim = [  x0_sim
                zeros(m,1)  ];
end

% Save this IC vector for post-learning simulation
x0_sim_f = x0_sim;

% Kronecker product integrals -- (x, x)
x0_sim = [  x0_sim
            zeros(nxztotb,1)  ];

% Kronecker product integrals -- (x_k, w_k)
x0_sim = [  x0_sim
            zeros(nxzbtot,1)  ];

% Kronecker product integrals -- (x_k, \tilde{w}_k)
x0_sim = [  x0_sim
            zeros(nxzbtot,1)  ];

% Kronecker product integrals -- (x_k, g_k(x)u)
x0_sim = [  x0_sim
            zeros(nxzbtot,1)  ];

% Output integrals \int y
x0_sim = [  x0_sim
            zeros(m,1)  ];

% *************************************************************************
% 
% DATA STORAGE
% 
% *************************************************************************

% Time vector, state trajectory, control signal
tlvec = [];
xlmat = [];
ulmat = [];
enmat = [];

% Weight storage
cmat_cell = cell(numloops,1);
for k = 1:numloops
    nxzbk = loop_cell{k}.nxzb;
    cmat_cell{k} = zeros(istarvec(k), nxzbk);
end

% Stores controllers, "P" matrices in each loop
P_cell = cell(numloops,1);
K_cell = cell(numloops,1);
for k = 1:numloops
    % Get number of states, outputs associated with this loop
    nxk = loop_cell{k}.nx;
    nyk = loop_cell{k}.ny;
    nxzk = loop_cell{k}.nxz;
    P_cell{k} = zeros(nxzk, nxzk, istarvec(k));
    K_cell{k} = zeros(nyk, nyk + nxk, istarvec(k)+1);
    K_cell{k}(:,:,1) = loop_cell{k}.K0;
end
u_sett.K_cell = K_cell;


% Stores previous sample
x0_cell = cell(numloops,1);
for k = 1:numloops
    x0_cell{k} = get_loop_state(x0_sim,0,k,xe_sim);
end

% Stores state trajectory samples at (base) sample instants
xTsmat = zeros(maxnsamp+1,nxztot);
xTsmat(1,:) = get_loop_state(x0_sim, 0, 0, xe_sim);

% Stores Kronecker product integrals -- (x, x)
Ibxmat = zeros(maxnsamp,nxztotb);

% Stores Kronecker product integrals -- (x_k, w_k)
IBxwmat = zeros(maxnsamp,nxzbtot);

% Stores Kronecker product integrals -- (x_k, \tilde{w}_k)
IBxtwmat = zeros(maxnsamp,nxzbtot);

% Stores Kronecker product integrals -- (x_k, g_k(x)u)
IBxgumat = zeros(maxnsamp,nxzbtot);


% ***********************
%       
% CONDITIONING DATA
%

% Stores condition number of LS "A" matrix at each iteration in each loop
cond_A_vec_cell = cell(numloops,1);
for k = 1:numloops
    cond_A_vec_cell{k} = zeros(istarvec(k),1);
end


% ***********************
%       
% MATRIX RELATING B(x,y) TO kron(x,y)
%   
%   B(x,y) = M kron(x,y)
%
M_cell = cell(numloops,1);
Mr_cell = cell(numloops,1);
for k = 1:numloops
    % Get number of states, outputs associated with this loop
%     nxk = loop_cell{k}.nx;
%     nyk = loop_cell{k}.ny;
    nxzk = loop_cell{k}.nxz;
%     nxzbk = loop_cell{k}.nxzb;
    [Mk, Mrk] = make_M(nxzk);
    M_cell{k} = Mk;
    Mr_cell{k} = Mrk;
end

% Make for the overall DIRL system
[Mtot, Mrtot] = make_M(nxztot);


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

% If learning is desired, execute it
if dolearning

% *************************************************************************
% *************************************************************************
%
% DATA COLLECTION
% 
% *************************************************************************
% *************************************************************************

% Keeps track of iteration count in each loop
ivec = ones(numloops,1);
u_sett.ivec = ivec;


for sampcnt = 1:maxnsamp

%     % Display sample count
%     disp(['RUNNING SAMPLE   ' num2str(sampcnt) ...
%         '     OUT OF  ' num2str(maxnsamp)])

    % ***********************
    %       
    % RUN SIMULATION
    %
    
    % Time span for simulation
    tspan = [sampcnt-1, sampcnt] * Ts;
    
    % Run simulation
    [t, x] = ode45(@odefunct, tspan, x0_sim);
    
%     % DEBUGGING: Plot states while calling ode45
%     options = odeset('OutputFcn',@odeplot);
%     figure(100);
%     [t, x] = ode45(@odefunct, tspan, x0_sim, options); 

    if dispxl

            % DEBUGGING: Plot state trajectory
            figure(100)
            plot(t,x(:,model.indV));
            title('State Trajectory -- V')
            grid on
            xlabel('Time (sec)')
            ylabel('V(t)') 
            hold on
            figure(101)
            plot(t,R2D*x(:,model.indg));
            title('State Trajectory -- gamma')
            grid on
            xlabel('Time (sec)')
            ylabel('gamma(t) (deg)') 
            hold on
            figure(102)
            plot(t,R2D*x(:,model.indq));
            title('State Trajectory -- q')
            grid on
            xlabel('Time (sec)')
            ylabel('q(t) (deg/s)') 
            hold on

            % DEBUGGING: Plot transformed states V subsystem
            ktmp = 1;
            nxkt = loop_cell{ktmp}.nx; 
            nykt = loop_cell{ktmp}.ny; 
            ltvec = length(t);
            xkmat = zeros(ltvec,nxkt+nykt);
            for tct = 1:length(t)
                xktmp = get_loop_state(x(tct,:)',t(tct),ktmp,xe_sim);
                xkmat(tct,:) = xktmp';
            end
            figure(200)
            plot(t,xkmat);
            title('State Trajectory -- V States')
            grid on
            xlabel('Time (sec)')
            ylabel('x(t)') 
            hold on

            % DEBUGGING: Plot transformed states \gamma subsystem
            ktmp = 2;
            nxkt = loop_cell{ktmp}.nx; 
            nykt = loop_cell{ktmp}.ny; 
            ltvec = length(t);
            xkmat = zeros(ltvec,nxkt+nykt);
            for tct = 1:length(t)
                xktmp = get_loop_state(x(tct,:)',t(tct),ktmp,xe_sim);
                xkmat(tct,:) = xktmp';
            end
            figure(201)
            plot(t,xkmat);
            title('State Trajectory -- \gamma States')
            grid on
            xlabel('Time (sec)')
            ylabel('x(t)') 
            hold on

    end

    % ***********************
    %       
    % DATA STORAGE
    %

    % Store time, state, control data
    tlvec = [    tlvec
                t       ];    
    xlmat = [    xlmat
                x       ];
    [utmp, entmp] = uxt_all(x,t);
    ulmat = [    ulmat
                utmp    ];    
    enmat = [    enmat
                entmp    ];       

    if dispxl

            % DEBUGGING: Plot control
            figure(300)
            plot(t,utmp(:,1));
            title('Control -- dT')
            grid on
            xlabel('Time (sec)')
            ylabel('dT(t)') 
            hold on
            figure(301)
            plot(t,R2D*utmp(:,2));
            title('Control -- dE')
            grid on
            xlabel('Time (sec)')
            ylabel('dE(t) (deg)') 
            hold on 

            % DEBUGGING: Plot noise
            figure(303)
            plot(t,entmp(:,1));
            title('Exploration noise  -- e_{dT}')
            grid on
            xlabel('Time (sec)')
            ylabel('e_{dT}(t)') 
            hold on
            figure(304)
            plot(t,R2D*entmp(:,2));
            title('Exploration noise -- e_{dE}')
            grid on
            xlabel('Time (sec)')
            ylabel('e_{dE}(t) (deg)') 
            hold on 

    end

    % Set up ICs for next simulation
    x1 = x(end,:)';
    t0 = t(1);
    t1 = t(end);

    % ***********************
    %       
    % STORE REGRESSION DATA
    %

    % Store current state trajectory data
    xTsmat(sampcnt+1,:) = get_loop_state(x1, t1, 0, xe_sim);

    % Store Kronecker product integrals -- (x, x)
    Ibxmat(sampcnt,:) = x1(indsIbx);
    
    % Store Kronecker product integrals -- (x_k, w_k)
    IBxwmat(sampcnt,:) = x1(indsIBxw{numloops+1});

    % Store Kronecker product integrals -- (x_k, \tilde{w}_k)
    IBxtwmat(sampcnt,:) = x1(indsIBxtw{numloops+1});

    % Store Kronecker product integrals -- (x_k, g_k(x) u)
    IBxgumat(sampcnt,:) = x1(indsIBxgu{numloops+1});

    % ***********************
    %       
    % RESET KRONECKER PRODUCT INTEGRALS
    %

    % Reset Kronecker product integrals -- (x, x)
    x1(indsIbx) = 0;

    % Reset Kronecker product integrals -- (x_k, w_k)
    x1(indsIBxw{numloops+1}) = 0;

    % Reset Kronecker product integrals -- (x_k, \tilde{w}_k)
    x1(indsIBxtw{numloops+1}) = 0;
    
    % Reset Kronecker product integrals -- (x_k, g_k(x)u)
    x1(indsIBxgu{numloops+1}) = 0;

    % Set up ICs for next simulation
    x0_sim = x1;

end         % END MAIN LOOP


% *************************************************************************
% *************************************************************************
%
% LEARNING
% 
% *************************************************************************
% *************************************************************************


% Learn in each loop
for k = 1:numloops

% If loop is active, perform learning
if doloopvec(k)

    % ***********************
    %       
    % GET REGRESSION DATA
    %    

    % Get number of states, outputs associated with this loop
    nxk = loop_cell{k}.nx;
    nyk = loop_cell{k}.ny;
    nxzk = loop_cell{k}.nxz;
    nxzbk = loop_cell{k}.nxzb;

    % Number of base sample period to collect, number of samples
    nTsk = nTsvec(k);
    lk = lvec(k);

    % Identity matrix of dimension n_k
    Ink = eye(nxzk);

    % Indices of states/outputs corresponding to this loop
    indsxirlk = indsxirl{k};
    indsyk = loop_cell{k}.indsy;

    % B_{k,k}
    Bkk = B_cell_sim{k,k};

    % Q_k, R_k
    Qk = loop_cell{k}.Q;
    Rk = loop_cell{k}.R;

    % Matrix relating B(x,y) to kron(x,y) in this loop
    Mk = M_cell{k};

    % Matrix relating kron(x, x) to B(x, x) in this loop
    Mrk = Mr_cell{k};

    % Matrix \delta_{x_k x_k}
    dxkxk = zeros(lk,nxzbk);
    xknTs0 = xTsmat(1,indsxirlk)';
    for scnt = 1:lk

        % Index of next sample
        indTs1 = scnt * nTsk + 1;

        % Next sample
        xknTs1 = xTsmat(indTs1,indsxirlk)';

        % B(x(t_1)+x(t_0), x(t_1)-x(t_0))
        Bx1mx0 = blf(xknTs1+xknTs0,xknTs1-xknTs0);
        
        % Store data
        dxkxk(scnt,:) = Bx1mx0';

        % Store next sample as previous
        xknTs0 = xknTs1;

    end

    % Kronecker product integral matrices
    Ixx_nkTs = zeros(lk,nxztot^2);
    indsIBxwlock = indsIBxwloc{k};
    IBxkwk_nkTs = zeros(lk,nxzbk);
    IBxktwk_nkTs = zeros(lk,nxzbk);
    IBxkguk_nkTs = zeros(lk,nxzbk);

    for scnt = 1:lk

        % Temp storage
        Ixx_nkTstmp = zeros(nxztot^2,1);
        IBxw_nkTstmp = zeros(nxzbk,1);
        IBxtw_nkTstmp = zeros(nxzbk,1);
        IBxgu_nkTstmp = zeros(nxzbk,1);

        for j = 1:nTsk

            % Current index
            ind = (scnt-1)*nTsk + j;

            % Extract integrals over current period
            Ibxtmp = Ibxmat(ind,:)';
            IBxwtmp = IBxwmat(ind,indsIBxwlock)';
            IBxtwtmp = IBxtwmat(ind,indsIBxwlock)';
            IBxgutmp = IBxgumat(ind,indsIBxwlock)';

            % For (x, x) integrals -- go from \bar{x} to kron(x,x)
            Ixxtmp = Mrtot * Ibxtmp;

            % Add these integrals to the total
            Ixx_nkTstmp = Ixx_nkTstmp + Ixxtmp;
            IBxw_nkTstmp = IBxw_nkTstmp + IBxwtmp;
            IBxtw_nkTstmp = IBxtw_nkTstmp + IBxtwtmp;
            IBxgu_nkTstmp = IBxgu_nkTstmp + IBxgutmp;

        end

        % Store in matrix
        Ixx_nkTs(scnt,:) = Ixx_nkTstmp;
        IBxkwk_nkTs(scnt,:) = IBxw_nkTstmp;
        IBxktwk_nkTs(scnt,:) = IBxtw_nkTstmp;
        IBxkguk_nkTs(scnt,:) = IBxgu_nkTstmp;


    end

    % ***********************
    %       
    % CALCULATE REGRESSION MATRICES
    %    

    % Indices of all loops besides current one
    if dox3
        indsall = (1:numloops+1)';
    else
        indsall = (1:numloops)';
    end
    indsnotk = indsall(indsall~=k);
    nindsall = size(indsall,1);
    nindsnotk = size(indsnotk,1);

    % Calculate matrices I_{x_k x_j}
    Ixkxj_cell = cell(nindsall,1);
    IBxkxk = zeros(lk, nxzbk);
    for j = indsall'
        indsxirlj = indsxirl{j};
        nxzj = size(indsxirlj,1);
        Ixkxj = subvec(Ixx_nkTs', nxztot, nxztot, indsxirlk, indsxirlj)';
        Ixkxj_cell{j} = Ixkxj;
        if j == k
            IBxkxk = Ixkxj * Mk';
        end
    end


    % ***********************
    %       
    % LEARNING LOOP
    %        

    % Get the matrix I_{x_k x_k}
    Ixkxk = Ixkxj_cell{k};
    
    for ik = 1:istarvec(k)

        % Get i + 1
        ikp1 = ik + 1;

        % Get current loop controller K_{i,k}
        Kik = K_cell{k}(:,:,ik);

        % Calculate Q_{i,k} = Q_k + K_{i,k}^T R_{k} K_{i,k}
        Qik = Qk + Kik' * Rk * Kik;

        % Calculate the matrix M_{i,k}
        Mik = Mk * kron(Ink,Bkk*Kik) * Mrk;

        % Calculate matrix I_{B(x_k, x_k)} * M_{i,k}^T
        IBxkxkMik = IBxkxk * Mik';
        
%         % Calculate matrix I_{x_k x_k} * kron(I_{n_k}, (B_{kk} K_{i,k})^T)
%         IxkxkBkkKik = Ixkxk * kron(Ink,(Bkk*Kik)');

        % FINAL REGRESSION MATRIX
%         ulAlsq = dxkxk - 2 * (IBxkguk_nkTs + IxkxkBkkKik * Mk');
        Aulsq = - 2 * (IBxkguk_nkTs + IBxkxkMik);
        ulAlsq = dxkxk + Aulsq;
        Alsq_sim = ulAlsq - 2 * IBxkwk_nkTs;
        Alsq = ulAlsq - 2 * IBxktwk_nkTs;        

        % FINAL REGRESSION VECTOR
%         blsq = - Ixkxk * Qik(:);
        blsq = - IBxkxk * vecsym(Qik);

        % LEAST-SQUARES REGRESSION
        cveck = Alsq \ blsq;

        % Calculate corresponding symmetric matrix
        Pik = invvecsym(cveck);

        % Calculate new controller
        Kik = Rk \ (Bkk' * Pik);

%         % DEBUGGING: Test different regressions
%         csim = Alsq_sim \ blsq;
%         Piksim = invvecsym(csim);
%         Kiksim = Rk \ (Bkk' * Piksim)
%         cul = ulAlsq \ blsq;
%         Pikul = invvecsym(cul);
%         Kikul = Rk \ (Bkk' * Pikul)
%         cKi = (-2 * IBxkxkMik) \ blsq;
%         PikKi = invvecsym(cKi);
%         KikKi = Rk \ (Bkk' * PikKi)
%         cu = Aulsq \ blsq;
%         Piku = invvecsym(cu);
%         Kiku = Rk \ (Bkk' * Piku)               
%         cuw = (Aulsq - 2 * IBxktwk_nkTs) \ blsq;
%         Pikuw = invvecsym(cuw);
%         Kikuw = Rk \ (Bkk' * Pikuw)        

        % Store weights
        cmat_cell{k}(ik,:) = cveck';

        % Store corresponding "P" matrix
        P_cell{k}(:,:,ik) = Pik;

        % Store contoller
        K_cell{k}(:,:,ikp1) = Kik;

        % Store conditioning data
        cond_A_vec_cell{k}(ik) = cond(Alsq);

        % Add one to PI index of this loop
        ivec(k) = ikp1;

        % Update controller in global settings
        u_sett.K_cell = K_cell;

        % Update the PI iteration counter in the global settings
        u_sett.ivec = ivec;

        % LEAST-SQUARES REGRESSION: PERTURBATION TERMS
        IBxkwkmtwk_nkTs = IBxkwk_nkTs - IBxktwk_nkTs;

        % DEBUGGING: Display PI iteration, controller, condition number
        disp(['  PI ITERATION IN EACH LOOP:  '   num2str(ivec')])
        disp(['cond(\tilde{A}) =        ' num2str(cond(Alsq))])
        disp(['||\tilde{A}|| =          ' num2str(norm(Alsq))])
        disp(['cond(A) =        ' num2str(cond(Alsq_sim))])
        disp(['||A|| =          ' num2str(norm(Alsq_sim))])     
        disp(['cond(\ul{A}) =        ' num2str(cond(ulAlsq))])
        disp(['||\ul{A}|| =          ' num2str(norm(ulAlsq))])   
        disp(['||\delta_{xx}|| =          ' num2str(norm(dxkxk))])  
        disp(['||I_{B(x,\tilde{w})}|| =          ' ...
            num2str(norm(IBxkwk_nkTs))])
        disp(['||I_{B(x,w)}|| =          ' num2str(norm(IBxkwk_nkTs))])  
        disp(['||I_{B(x,w -\tilde{w})}|| =          ' ...
            num2str(norm(IBxkwkmtwk_nkTs))])     
        disp(['||I_{B(x,g(x)u)}|| =          ' ...
            num2str(norm(IBxkguk_nkTs))])
        disp(['||I_{B(x,BK_ix)}|| =          ' ...
            num2str(norm(IBxkxkMik))])
%             num2str(norm((IxkxkBkkKik) * Mk'))])        

        Kik

        % DEBUGGING: Display final controller error
        if ik == istarvec(k)
            switch numloops
                case 1
                    Kkstar = lq_data_opt_sim.Kcirl;
                case 2
                    switch k
                        case 1
                            Kkstar = lq_data_opt_sim.lq_data_11.K;
                        case 2
                            Kkstar = lq_data_opt_sim.lq_data_22.K;
                    end
            end
            eKk = Kik - Kkstar;
            normeKk = norm(eKk);
            disp(['||K_{i*,j} - K_j^*|| =   ' num2str(normeKk)])
        end

    end
    


end

end

% DEBUGGING: Print done learning
disp('***** LEARNING COMPLETE *****')



end                 % END LEARNING PORTION

%%
% *************************************************************************
% *************************************************************************
%
% LOAD, FORMAT FINAL CONTROLLER
% 
% *************************************************************************
% *************************************************************************

switch numloops


    % ***********************
    %       
    % 1 LOOP
    %    

    case 1

        % ***********************
        %       
        % LOAD
        %       

        if dolearning
        
            % Final controller -- IRL state partition
            Kirl = K_cell{1}(:,:,end);
        
        else
        
            % Load final controller -- IRL state partition
            dirl_data = load([relpath_data filename_data]);
            dirl_data = dirl_data.dirl_data;
            lq_data = dirl_data.lq_data;
            Kirl = lq_data.Kirl;
        
        end        

        % ***********************
        %       
        % FORMAT
        %       
 
        % Individual controllers: z, y, x_r
        % IRL state partition -> LQ servo state partition        
        Kz = Kirl(:,[1 3]);
        Ky = Kirl(:,[2 4]);
        Kr = Kirl(:,[5 6]);

        % Form composite controller
        K = [   Kz  Ky  Kr];


        % ***********************
        %       
        % STORE
        %       

        % Store composite controller -- IRL state partition
        lq_data.Kirl = Kirl;


    % ***********************
    %       
    % 2 LOOPS
    %     

    case 2

        % ***********************
        %       
        % LOAD
        %       

        if dolearning
        
            % Final controllers
            KV = K_cell{1}(:,:,end);
            Kg = K_cell{2}(:,:,end);
        
        else
        
            % Load final controllers
            dirl_data = load([relpath_data filename_data]);
            dirl_data = dirl_data.dirl_data;
            lq_data = dirl_data.lq_data;
            KV = lq_data.lq_data_V.K;
            Kg = lq_data.lq_data_g.K;
        
        end

        % ***********************
        %       
        % FORMAT
        %       

        % Extract parts of feedback gain matrix K corresponding to each of
        % the states in the partition
        m1 = loop_cell{1}.ny;
        nx1 = loop_cell{1}.nx;
        KzV = KV(:,1:m1);
        KyV = KV(:,m1+1:2*m1);
        % KrV = KV(:,2*m1+1:m1+nx1);
        
        m2 = loop_cell{2}.ny;
        nx2 = loop_cell{2}.nx;
        Kzg = Kg(:,1:m2);
        Kyg = Kg(:,m2+1:2*m2);
        Krg = Kg(:,2*m2+1:m2+nx2);
        
        % Form composite controller
        K = [   KzV     0       KyV     0       zeros(1,2)  
                0       Kzg     0       Kyg     Krg         ];

        % Individual controllers: z, y, x_r
        Kz = K(:,1:m);
        Ky = K(:,m+1:2*m);
        Kr = K(:,2*m+1:m+n-1);

        % ***********************
        %       
        % STORE
        %       

        % Store V controller
        lq_data_V.K = KV;
        lq_data_V.Kz = KzV;
        lq_data_V.Ky = KyV;
        
        % Store \gamma controller
        lq_data_g.K = Kg;
        lq_data_g.Kz = Kzg;
        lq_data_g.Ky = Kyg;
        lq_data_g.Kr = Krg;
        
        % Store individual loop data
        lq_data.lq_data_V = lq_data_V;
        lq_data.lq_data_g = lq_data_g;



end

% Store composite controller
lq_data.K = K;
lq_data.Kz = Kz;
lq_data.Ky = Ky;
lq_data.Kr = Kr; 

  
%%
% *************************************************************************
% *************************************************************************
%
% RUN SIMULATION WITH FINAL CONTROLLER
% 
% *************************************************************************
% *************************************************************************

if dopostlearning



% ***********************
%       
% RUN SIMULATION -- CALL LQ SERVO SIMULATOR
%

% Temp alg_settings struct
alg_settings_tmp = alg_settings;
alg_settings_tmp.alg = 'wang_stengel_2000_lq_servo_inout';
alg_settings_tmp.K = K;
alg_settings_tmp.model_nom_tag = model_sim_tag;

% Run LQ servo simulation
out_data_tmp = alg_wang_stengel_2000_lq_servo_inout(alg_settings_tmp);


% DEBUGGING: Print done simulating
disp('***** POST-LEARNING SIMULATION COMPLETE *****')

end


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

% Post-learning data
if dopostlearning

    % Store control signal
    out_data.umat = out_data_tmp.umat;
    
    % Time vector
    out_data.tvec = out_data_tmp.tvec;
    
    % Store post-learning state vector
    out_data.xmat = out_data_tmp.xmat;

end

% LQ data
out_data.lq_data = lq_data;

% Loop data
out_data.loopdata = loopdata;

% Store indices
out_data.inds = inds;

% Learning data
if dolearning

    % Control signal learning phase
    out_data.ulmat = ulmat;
    
    % Time vector learning phase
    out_data.tlvec = tlvec;

    % State vector learning phase
    out_data.xlmat = xlmat;

    % IRL learning data
    out_data.cmat_cell = cmat_cell;
    out_data.P_cell = P_cell;
    out_data.K_cell = K_cell;
    
    % Conditioning data
    out_data.cond_A_vec_cell = cond_A_vec_cell;

else

    % Control signal learning phase
    out_data.ulmat = dirl_data.ulmat;
    
    % Time vector learning phase
    out_data.tlvec = dirl_data.tlvec;

    % State vector learning phase
    out_data.xlmat = dirl_data.xlmat;
    
    % IRL learning data
    out_data.cmat_cell = dirl_data.cmat_cell;
    out_data.P_cell = dirl_data.P_cell;
    out_data.K_cell = dirl_data.K_cell;
    
    % Conditioning data
    out_data.cond_A_vec_cell = dirl_data.cond_A_vec_cell;

end

% Optimal LQ data -- simulation model
out_data.lq_data_opt_sim = lq_data_opt_sim;

%%
% *************************************************************************
% *************************************************************************
%
% SAVE DIRL LEARNING DATA
%
% *************************************************************************
% *************************************************************************

if dolearning

    % LQ data
    dirl_data.lq_data = lq_data;
    
    % Loop data
    dirl_data.loopdata = loopdata;
    
    % Control signal learning phase
    dirl_data.ulmat = ulmat;
    
    % Time vector learning phase
    dirl_data.tlvec = tlvec;
    
    % State vector learning phase
    dirl_data.xlmat = xlmat;
    
    % IRL learning data
    dirl_data.cmat_cell = cmat_cell;
    dirl_data.P_cell = P_cell;
    dirl_data.K_cell = K_cell;
    
    % Conditioning data
    dirl_data.cond_A_vec_cell = cond_A_vec_cell;
    
    % Overwrite current controller if desired
    if updatecontroller
    
        % Save data 
        varname = 'dirl_data';
        save([relpath_data_dirl filename_data_dirl], varname);
    
    end

end

% Store DIRL data
out_data.dirl_data = dirl_data;



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

% Extract nominal model
model_nom = u_sett.model_nom;

% Extract simulation model
model_sim = u_sett.model_sim;

% Get coordinate transformations
% sx = u_sett.sx;
sxh = u_sett.sxh;
su = u_sett.su;
sy = u_sett.sy;

% Get indices of state partition
inds = u_sett.inds;

% Get loop settings
loop_cell = u_sett.loop_cell;

% Initialize empty state derivative vector
xdot = zeros(u_sett.n_sim, 1);

% Extract plant states
xp = x(inds.indsx);

% % Get equilibrium point x_e (pre-transformation)
% xe = u_sett.xe;
% 
% % Get equilibrium control u_e (pre-transformation)
% ue = u_sett.ue;

% Get equilibrium point x_e (pre-transformation) -- nominal system
xe_nom = u_sett.xe_nom;

% Get equilibrium point x_e (pre-transformation) -- simulated system
xe = u_sett.xe_sim;

% Get equilibrium control u_e (pre-transformation) -- simulated system
ue = u_sett.ue_sim;


% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% Get state of linear system (pre-transformation) -- nominal system
% \tilde{x} = x - x_e
tx_nom = xp - xe_nom;

% Evaluate drift dynamics
if u_sett.lin1nonlin0
    % System linear
    f_x = model_sim.lin.ApdE * tx;
else
    % System nonlinear
    f_x = model_sim.fx(xp);
end

% Evaluate input gain matrix
if u_sett.lin1nonlin0
    % System linear
    g_x = model_sim.lin.BpdE;
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

% Calculate \tilde{u} = u - u_{e} (pre-transformation) 
tu = u - ue;


% Evaluate reference trajectory r(t) (pre-transformation)
if u_sett.islearning
    rt = eval_xr(t, r_sett.r_sett_train);
else
    rt = eval_xr(t, r_sett);
end

% Evaluate reference trajectory r(t) (post-transformation)
yr = [rt(1); rt(5)];
yrp = sy * yr;

% Get y(t) (post-transformation)
yp = sy * x(inds.indsxr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(inds.indsxr);

% Get (small-signal) reference command r (post-transformation)
trp = yrp - yep;

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% If prefilters used, get filtered reference command (post-transformation)
if u_sett.pf1nopf0
    trfp = x(inds.indspf);
end

% Evaluate integrator state derivative \dot{z} (post-transformation)
if u_sett.pf1nopf0 && ~ u_sett.islearning
    zdot = -(trfp - typ);
else
    zdot = -(trp - typ);
end

% State derivatives
if u_sett.lin1nonlin0
    dx = f_x + g_x * tu;
else
    dx = f_x + g_x * u;
end
xdot(inds.indsx) = dx;

% Append integral augmentation state derivatives
xdot(inds.indsz) = zdot;

% If prefilter inserted, evaluate prefilter dynamics (post-transformation)
if u_sett.pf1nopf0
    pfdot = -diag(u_sett.pfavec) * trfp + diag(u_sett.pfavec) * trp;
    xdot(inds.indspf) = pfdot;
end      

% ***********************
%       
% KRONECKER PRODUCT INTEGRALS -- LEARNING PHASE ONLY
%

if u_sett.islearning

    % Get aggregate state
    xirl = get_loop_state(x,t,0,xe);

    % Kronecker product integrals -- (x, x)
    dIbx = blf(xirl,xirl);

    xdot(inds.indsIbx) = dIbx;

    % Integrals -- (x_k, w_k), (x_k, \tilde{w}_k), (x_k, g_k(x) u)

    % Evaluate control portion of derivative
    if u_sett.lin1nonlin0
        gxu = g_x * tu;
    else
        gxu = g_x * u;
    end

    % Transformed control portion of state derivative
    gxup = sxh * gxu;

    for k = 1:u_sett.numloops

        % Get x_k (post-trans) -- simulation system
        xk = get_loop_state(x,t,k,xe);

        % Calculate w_k = f_k(x) - A_{kk} x_k
        wk = get_w(x, t, k, model_sim, model_sim);

        % Calculate 
        % \tilde{w}_k = \tilde{f}_k(x) - \tilde{A}_{kk} \tilde{x}_k
        twk = get_w(x, t, k, model_nom, model_sim);

        % Calculate g_{k}(x) u
        switch u_sett.numloops
            case 1
                gxupk = [   0
                            gxup(1)
                            0
                            gxup(2:4) ];
            case 2
                gxupk = [   zeros(loop_cell{k}.ny)
                            gxup(loop_cell{k}.indsx) ];
        end

        % Calculate B(x_k, w_k)
        dIBxkwk = blf(xk,wk);

        % Calculate B(x_k, \tilde{w}_k)
        dIBxktwk = blf(xk,twk);

        % Calculate B(x_k, g_{k}(x) u)
        dIBxkguk = blf(xk,gxupk);

        % Store derivative terms
        xdot(inds.indsIBxw{k}) = dIBxkwk;
        xdot(inds.indsIBxtw{k}) = dIBxktwk;
        xdot(inds.indsIBxgu{k}) = dIBxkguk;

    end


    % Output integrals \int y
    xdot(inds.indsIy) = typ;


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

function [u, nt] = uxt_alg(x, t)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Control settings
global r_sett;
global u_sett;


% Get coordinate transformations
% sx = u_sett.sx;
su = u_sett.su;
sy = u_sett.sy;

% Get loop settings
loop_cell = u_sett.loop_cell;

% Get indices of state partition
inds = u_sett.inds;

% Get equilibrium point x_e (pre-transformation) -- simulated system
xe = u_sett.xe_sim;

% Get equilibrium control u_e (pre-transformation) -- simulated system
ue = u_sett.ue_sim;


% Evaluate exploration noise (post-transformation)
ntp = eval_noise(t, m, u_sett.noise);


% Calculate control in each loop
tup = zeros(m,1);
for k = 1:u_sett.numloops

    % Get indices of state, output vector to pull
    indsxk = loop_cell{k}.indsx;
    indsyk = loop_cell{k}.indsy;

    % Get number of states, outputs associated with this loop
    nxk = loop_cell{k}.nx;
    nyk = loop_cell{k}.ny;

    % Get PI iteration number of current loop
    ik = u_sett.ivec(k);

    % Get the state vector corresponding to this loop
    [xk, zk, ek] = get_loop_state(x, t, k, xe);


    % Get rest of state associated with this loop
    txrk = xk(2*nyk+1:nyk+nxk);

    % Get contoller
    Kk = u_sett.K_cell{k}(:,:,ik);
    
    % Extract parts of feedback gain matrix K corresponding to each of
    % the states in the partition
    switch u_sett.numloops
        case 1
            Kz = Kk(:,[1 3]);
            Ky = Kk(:,[2 4]);
            Kr = Kk(:,[5 6]);
        case 2
            Kz = Kk(:,1:nyk);
            Ky = Kk(:,nyk+1:2*nyk);
            Kr = Kk(:,2*nyk+1:nyk+nxk);
    end

    
    % Calculate control (post-transformation) 
%     tup(indsyk) = Kz * (-zk) + Ky * ek - Kr * txrk;
    tup(indsyk) = Kz * (-zk) + Ky * (-ek) - Kr * txrk;
    
end

% Insert probing noise if desired
if u_sett.islearning
    tup = tup + ntp;
end

% Calculate linear control (pre-tranformation)
tu = su \ tup;

% Calculate exploration noise (pre-transformation)
nt = su \ ntp;

% Calculate final control u = u_e + \tilde{u}
u = ue + tu;    


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE CONTROL SIGNAL u(x, t) FOR STRING OF SAMPLES
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function [umat, enmat] = uxt_all(xmat, tvec)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Number of data points
ltvec = size(tvec,1);

% Initialize empty matrix
umat = zeros(ltvec, m);
enmat = zeros(ltvec, m);

% Calculate control
for k = 1:ltvec
    
    % Get time
    t = tvec(k);

    % Get state vector 
    xs = xmat(k,:)';
    
    % Evaluate control 
    [u, en] = uxt_alg(xs, t);
                
    % Store control
    umat(k,:) = u';

    % Store noise
    enmat(k,:) = en';

end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% GET PORTION OF STATE VECTOR CORRESPONDING TO THE CURRENT LOOP
%
% (Note: After shifting by trim and performing coordinate transformation)
%
% State partition of input:
%
%   x = [   x_p     Plant states (pre-transformation) \in R^{n}
%           z       Integrator states \in R^{m}
%           x_{pf}  (IF APPLICABLE) Pre-filter states \in R^{m}
%           x_{ir}  Integral reinforcement functions \in R^{'numloops'} ]
%
% State partition of output
%   
%   \tilde{x}_{k}^{'} =
%       [   \tilde{x}_{v,k}^{'} Plant states (shifted by trim,
%                               post-transformation
%           z_{k}               Integrator states applicable to this loop
%           x_{pf,k}            Pre-filter states applicable to this loop ]
%           
%       
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

function [xk, zk, ek] = get_loop_state(x, t, k, xe)

% Global variables
global sys;
n = sys.n;
m = sys.m;

% Control settings
global r_sett;
global u_sett;

% Get coordinate transformations
sx = u_sett.sx;
sxh = u_sett.sxh;
% su = u_sett.su;
sy = u_sett.sy;

% Indices of state vector partition
inds = u_sett.inds;

% Get loop settings
loop_cell = u_sett.loop_cell;

% Get loop settings -- master partition
loop_cell_m = u_sett.loop_cell_m;

% Extract plant states x_p (pre-transformation)
xp = x(inds.indsx);

% % Get equilibrium point x_e (pre-transformation)
% xe = u_sett.xe;

% % Get equilibrium point x_e (pre-transformation) -- simulated system
% xe = u_sett.xe_sim;

% Evaluate reference trajectory r(t) (pre-transformation)
if u_sett.islearning
    rt = eval_xr(t, r_sett.r_sett_train);
else
    rt = eval_xr(t, r_sett);
end

% Evaluate reference trajectory r(t) (post-transformation)
yr = [rt(1); rt(5)];
yrp = sy * yr;

% Get y(t) (post-transformation)
yp = sy * x(inds.indsxr);

% Get equilibrium value of the output y_{e} (post-transformation)
yep = sy * xe(inds.indsxr);

% Get (small-signal) reference command r (post-transformation)
trp = yrp - yep;

% Get (small-signal) output y (post-transformation)
typ = yp - yep;

% If prefilters used, get filtered reference command (post-transformation)
if u_sett.pf1nopf0
    trfp = x(inds.indspf);
end

% Calculate (small-signal) tracking error e = y - r (post-transformation)
if u_sett.pf1nopf0 && ~u_sett.islearning
    e = -(trfp - typ);
else
    e = -(trp - typ);
end


% Extract integral errors (post-transformation)
zp = x(inds.indsz);

% Extract output integrals \int y (post-transformation)
if u_sett.islearning
    Ityp = x(inds.indsIy);
else
    Ityp = zeros(m,1);
end


% Get state of linear system (pre-transformation) 
% \tilde{x} = x - x_e
tx = xp - xe;

% % If system has height mode, take out the height mode
% tx = tx([1:sys.model.indh-1 sys.model.indh+1:n]);

% Apply coordinate transformation
% x' = S_x x
%    = [    y
%           x_r
%           h       ]
txp = sxh * tx;


% Check if it is desired to pull entire state or not
doall = k == 0;

% Initialize empty output vector
xk = [];
zk = [];
ek = [];

for kj = 1:u_sett.numloops_m

    % Get indices of state, output vector to pull
    indsxk = loop_cell_m{kj}.indsx;
    indsyk = loop_cell_m{kj}.indsy;
    % indsIntk = loop_cell{k}.indsInt;
    
    % Get number of states, outputs associated with this loop
    nxk = loop_cell_m{kj}.nx;
    nyk = loop_cell_m{kj}.ny;
    
    
    % Pull appropriate state indices
    txpk = txp(indsxk);
    zpk = zp(indsyk);
    Iyk = Ityp(indsyk);
    tepk = e(indsyk);
    typk = typ(indsyk);
    % if u_sett.pf1nopf0
    %     trfk = trf(indsyk);
    % end
    
    % Stack output vector
    % xk = [  zk
    %         txpk ];
    xk = [  xk
%             zk
            Iyk
%             tepk
            typk
            txpk(nyk+1:end)];

    zk = [  zk
            zpk ];
    ek = [  ek
            tepk ];

end

% If a specific loop has been selected, pull the data from that loop
if ~doall

    % Get indices of thus loop in the IRL state partition
    indsirlxk = inds.indsxirl{k}; 

    % Get indices of the output vector in this loop
    indsyk = loop_cell{k}.indsy;

    % Extract loop data
    xk = xk(indsirlxk);
    zk = zk(indsyk);
    ek = ek(indsyk);

end


% If x_3 loop is done, add on h
if doall && u_sett.dox3
    xk = [  xk
            txp(end)];
end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALCULATE w_k = f_k(x) - A_{kk} x_k
%                
% *************************************************************************
% *************************************************************************
% *************************************************************************

function wk = get_w(x, t, k, model_nom, model_sim)


% Global variables
global sys;
n = sys.n;
m = sys.m;

% Control settings
global r_sett;
global u_sett;

% Get coordinate transformations
sx = u_sett.sx;
sxh = u_sett.sxh;
% su = u_sett.su;
sy = u_sett.sy;

% Indices of state vector partition
inds = u_sett.inds;

% Get loop settings
loop_cell = u_sett.loop_cell;
numloops = u_sett.numloops;


% Extract plant states
xp = x(inds.indsx);

% Get equilibrium point x_e (pre-transformation)
if u_sett.wnom1sim0
    xe = model_nom.trimconds.xe_dE;
else
    xe = model_sim.trimconds.xe_dE;
end

% % Get equilibrium control u_e (pre-transformation)
% ue = model_sim.trimconds.ue_dE;

% Evaluate drift dynamics
if u_sett.lin1nonlin0
    % System linear
    f_x = model_nom.lin.ApdE * tx;
else
    % System nonlinear
    f_x = model_nom.fx(xp);
end


% Transformed state derivative (with nominal drift dynamics)
fxp = sxh * f_x;

% Get A_{kk}
Akk = model_nom.A_cell{k,k};

% Get x_k (post-trans)
xk = get_loop_state(x,t,k,xe);

% Get f_k(x) (post-trans)
fxpk = fxp(loop_cell{k}.indsx);

% Calculate w_k = f_k(x) - A_{kk} x_k
switch numloops
    case 1
        wk = [  xk(2) ; fxpk(1) ; xk(4) ; fxpk(2:4)] - Akk * xk;
    case 2
        wk = [  xk(2) ; fxpk ] - Akk * xk;
end



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%       
% MAKE STATE-SPACE MATRICES FOR EACH LOOP
%   
% *************************************************************************
% *************************************************************************
% *************************************************************************

function [A_cell, B_cell] = make_AB_cells(lin)

% Global settings
global u_sett;

% "A", "B" IRL matrices -- nominal model
Airl = lin.io.Airl;
Birl = lin.io.Birl;

% Extract loop settings
dox3 = u_sett.dox3;
numloops = u_sett.numloops;
loop_cell = u_sett.loop_cell;
indsxirl = u_sett.inds.indsxirl;

if dox3
    A_cell = cell(numloops+1, numloops+1);
    B_cell = cell(numloops+1, numloops);
else
    A_cell = cell(numloops, numloops);
    B_cell = cell(numloops, numloops);
end

for k = 1:numloops
    indsxirlk = indsxirl{k};
    indsyk = loop_cell{k}.indsy;
    for j = 1:numloops
        indsxirlj = indsxirl{j};
        indsyj = loop_cell{j}.indsy;
        A_cell{k,j} = Airl(indsxirlk,indsxirlj);
        B_cell{k,j} = Birl(indsxirlk,indsyj);
    end
end

if dox3
    kx3 = numloops+1;
    indsxirlx3 = indsxirl{kx3};
    for j = 1:numloops
        indsxirlj = indsxirl{j};
        indsyj = loop_cell{j}.indsy;
        A_cell{kx3,j} = Airl(indsxirlx3,indsxirlj);
        A_cell{j,kx3} = Airl(indsxirlj,indsxirlx3);
        B_cell{kx3,j} = Birl(indsxirlx3,indsyj);
    end
    A_cell{kx3,kx3} = Airl(indsxirlx3,indsxirlx3);
end

