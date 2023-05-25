function out_data = alg_irl_old(alg_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INTEGRAL REINFORCEMENT LEARNING (IRL) ALGORITHM
%
% Brent Wallace  
%
% 2021-11-06
%
% This program implements the IRL algorithm presented in,
%
%   D. Vrabie and F.L. Lewis. "Neural network approach to continuous-time
%   direct adaptive optimal control for partially unknown nonlinear
%   systems." Neural Networks, 22:237-246, 2009.
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% out_data = alg_irl(alg_settings)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings  struct with the following fields:
%   
%   preset                  (String) example preset (see main.m for
%                           options).
%   sys                     (Struct) contains system tag/info. See notes in
%                           'config.m' for specific fields.
%   alg                     (String) redundant for this function. Contains
%                           the tag of this algorithm.
%   Q                       (n x n matrix, or string) If matrix, is the
%                           positive definite state penalty matrix. If
%                           string, is the tag of the desired non-quadratic
%                           positive definite state penalty function.
%   R                       (m x m matrix, or string) If matrix, is the
%                           positive definite control penalty matrix. If
%                           string, is the tag of the desired non-quadratic
%                           positive definite control penalty function.
%   basis                   (Struct) contains activation function basis
%                           parameters. Has the following fields:
%       .tag                (String) tag of the desired activation function
%                           basis to use (see eval_phi.m for options).
%       .N                  (Integer) the integer "N," or the basis
%                           dimension.
%   noise                   (Struct) contains info for probing noise. NOTE:
%                           Not required for this algorithm. Has the
%                           following fields:
%       .tag                (String) tag of specific probing noise signal
%                           to be injected (see eval_noise.m for options).
%                           If no noise is desired, simply enter this as
%                           '0'.
%   T                       (Double) integral reinforcement interval length
%                           (sec).
%   istar                   (Integer) number of policy iterations to
%                           execute before terminating the algorithm.
%   num_sims_per_iter       (Integer) Number of simulations to execute per
%                           iteration of the PI algorithm. NOTE: a
%                           simulation consists of 'l'
%                           data samples, each taken over 'T' seconds. So
%                           each iteration has 'num_sims_per_iter' *
%                           'l' total T-second samples
%                           collected.
%   l     (Integer) number of samples to collect per
%                           simulation.     
%   x0mat                   (Matrix) This matrix can be empty or have up to
%                           'istar' * 'num_sims_per_iter' rows, with
%                           n columns. Each time a new simulation begins,
%                           the initial conditions need to be set. These
%                           ICs can be set manually for as many of the
%                           total 'istar' * 'num_sims_per_iter'
%                           simulations run in the algorithm. If x0mat runs
%                           out of ICs, then the ICs will be generated
%                           either randomly at the beginning of each new
%                           simulation, or will be carried over from the
%                           previous simulation's final values (see
%                           variable x0_behavior for details).
%   x0_behavior             (String) Determines the manner in which ICs are
%                           generated at the beginning of each new
%                           simulation, after the algorithm has run out of
%                           ICs specified in x0mat. Has the options:
%       'rand'              Randomly generate new ICs.
%       'cont'              Continue ICs of new simulation as final values
%                           of previous simulation.
%   c_0                      (N-dimensional vector) ICs for critic NN, where
%                           N is the critic NN basis dimension.
%   tsim                    (Double) time window for post-learning
%                           simulation (sec). I.e., if learning happens
%                           over [0, t_f], post-learning happens over [t_f,
%                           t_f + tsim].
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% out_data                  (Struct) algorithm output data. Has the
%                           following fields:
%       .tvec               ('simlength'-dimensional vector) vector of time
%                           indices corresponding to the simulation time
%                           instants over the course of the algorithm
%                           execution.
%       .xmat               ('simlength' x n matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose n-columns are the state vector at the
%                           respective time instant.
%       .umat               ('simlength' x m matrix) Matrix whose row
%                           indexes the time instants specified in .tvec,
%                           and whose m-columns are the control signal u(t)
%                           at the respective time instant.
%       .tvec_pi            ('istar' - dimensional vector)
%                           Vector of time instants corresponding to the
%                           sample instants of each new iteration of the PI
%                           algorithm.
%       .c_mat               (N x 'istar' matrix) critic NN weights
%                           at each of the time instants of .tvec_pi.
%       cond_A_vec          ('istar'-dim. vector) The i-th index of
%                           this vector contains the condition number of
%                           the matrix involved in performing the
%                           least-squares minimization associated with the
%                           i-th iteration weight update.
%       
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

% System
sys = alg_settings.sys;             % System array
n = sys.n;                          % System order
m = sys.m;                          % System input dimension
model = sys.model;                  % System model


% Reference signal r(t) settings -- training phase
r_sett_train = alg_settings.r_sett_train;

% Reference signal r(t) settings
r_sett = alg_settings.r_sett;
r_sett.r_sett_train = r_sett_train;

% Exploration noise
noise = alg_settings.noise;   

% Base sample period (sec)
Ts = alg_settings.Ts;

% Probing noise
noise = alg_settings.noise; 

% Post-learning simulation length
tsim = alg_settings.tsim;

% Holds settings for each loop
loop_cell = alg_settings.loop_cell;

% Do x_3 loop (=1) or not (=0)
dox3 = alg_settings.dox3;

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

% "A", "B" IRL matrices -- nominal model
Airl = model_nom.lin.io.Airl;
Birl = model_nom.lin.io.Birl;

% Total "R" matrix
R = loop_cell{1}.R;

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

% Nominal model
u_sett.model_nom = model_nom;
u_sett.xe_nom = xe_nom;
u_sett.ue_nom = ue_nom;

% Simulation model
u_sett.model_sim = model_sim;
u_sett.xe_sim = xe_sim;
u_sett.ue_sim = ue_sim;

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

% % Trim
% u_sett.xe = xe;
% u_sett.ue = ue;

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
istar = istarvec(1);

% Number of samples to collect per iteration for each loop
lvec = zeros(numloops,1);
for k = 1:numloops
    lvec(k) = loop_cell{k}.l;
end
loopdata.lvec = lvec;
l = lvec(1);

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

% Integral reinforcement
len = 1;
indsintreinf = (cnt:cnt+len-1)';
cnt = cnt + len;

% TOTAL SIMULATION STATE VECTOR LENGTH
n_sim = cnt - 1;
u_sett.n_sim = n_sim;


% Store
inds.indsxr = indsxr;
inds.indsx = indsx;
inds.indsz = indsz;
if pf1nopf0
    inds.indspf = indspf;
end
inds.indsintreinf = indsintreinf;
u_sett.inds = inds;

% Store loop_cell struct in global settings
u_sett.loop_cell = loop_cell;
u_sett.numloops = numloops;

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



% ***********************
%       
% GET STATE-SPACE MATRICES FOR EACH LOOP
%   

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

% Store
u_sett.A_cell = A_cell;
u_sett.B_cell = B_cell;


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

% Integral reinforcement
x0_sim = [  x0_sim
            0       ];


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

% Stores integral reinforcement
V = zeros(l,1);

% Stores state trajectory samples at (base) sample instants
xTsmat = zeros(l+1,nxztot);
xTsmat(1,:) = get_loop_state(x0_sim, 0, 0, xe_sim);


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

% Weight storage
cmat_cell = cell(numloops,1);
for k = 1:numloops
    nxzbk = loop_cell{k}.nxzb;
    cmat_cell{k} = zeros(istarvec(k), nxzbk);
end

% ***********************
%       
% CONDITIONING DATA
%

% Stores condition number of LS "A" matrix at each iteration in each loop
cond_A_vec_cell = cell(numloops,1);
for k = 1:numloops
    cond_A_vec_cell{k} = zeros(istarvec(k),1);
end



% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


for i = 1:istar
% for i = 1:istar - 1          % DEBUGGING
        
    % *************************************************************
    %
    % SIMULATION LOOP
    %
    % This inner loop is the one in which the individual
    % simulations are run. One simulation will consist of
    % 'l' T-second simulations, each run
    % separately so that data may be collected at the end of the
    % T-second interval.
    %  
    for sampcnt = 1:l
        
        % ***********************
        %       
        % RUN SIMULATION
        %
        
        % Time span for current simulation
        tspan = ...
            Ts * (i - 1) * l +...
            Ts * [sampcnt - 1, sampcnt];

        % Run simulation
%             [t, x] = ode23(@odefunct, [0 T], x0);
        [t, x] = ode45(@odefunct, tspan, x0_sim);

%             % DEBUGGING: Plot state trajectory
%             figure(100)
%             plot(t,x(:,1:n));
%             title('State Trajectory')
%             grid on
%             xlabel('Time (sec)')
%             ylabel('x(t)') 
%             hold on
        
%             % DEBUGGING: Plot state trajectory
%             figcount = 200;
%             for xi = 1:n
%                 figure(figcount)
%                 hold on
%                 plot(t,x(:,xi));
%                 title(['State Trajectory x_{' num2str(xi) '}(t)'])
%                 figcount = figcount + 1;
%             end
      
        
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
        
        % ***********************
        %       
        % PREPARE NEXT SAMPLE
        % 
        
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

        % Extract integral reinforcement at end of simulation
        V(sampcnt,1) = x1(indsintreinf);            

        % IC for next T-second simulation
        % State is carried over from previous T-second simulation,
        % integral reinforcement is reset
        x0_sim = x1;
        x0_sim(indsintreinf) = 0;
        
    end


    % ***********************
    %       
    % LEAST SQUARES MINIMIZATION TO DETERMINE CRITIC NN WEIGHTS FOR NEXT
    % ITERATION


    % Matrix \delta_{x_k x_k}
    dxx = zeros(l,nxzbtot);
    xknTs0 = xTsmat(1,indsxirl{1})';
    for scnt = 1:l

        % Index of next sample
        indTs1 = scnt + 1;

        % Next sample
        xknTs1 = xTsmat(indTs1,indsxirl{1})';

        % B(x(t_1)+x(t_0), x(t_1)-x(t_0))
        Bx1mx0 = blf(xknTs1+xknTs0,xknTs1-xknTs0);
        
        % Store data
        dxx(scnt,:) = Bx1mx0';

        % Store next sample as previous
        xknTs0 = xknTs1;

    end


    
    % Least-squares minimization
    Alsq = dxx;
    ci = Alsq \ (-V);

    % Calculate corresponding symmetric matrix
    Pi = invvecsym(ci);

    % Calculate new controller
    Ki = R \ (B_cell{1,1}' * Pi);

    % Store new weight
    cmat_cell{1}(i,:) = ci';  

    % Store "P", "K" matrices
    P_cell{1}(:,:,i) = Pi;
    K_cell{1}(:,:,i+1) = Ki;
    
    % Store condition number of matrix involved in least-squares
    % minimization
    cond_A_vec_cell{k}(i) = cond(Alsq);
    
    % DEBUGGING: Show iteration count, norm difference
    disp('*****')
    disp(['i = ' num2str(i) ])   
    
    % DEBUGGING: Check problem conditioning
    disp(['Condition Number of "A" for Least Squares:           '...
                num2str(cond(Alsq), 4)])
    
end

% % DEBUGGING: Plot critic NN params
% figure(101)
% tmp = (0:1:istar) * T * num_sims_per_iter * l;
% plot(tmp, c_mat(:,1:istar + 1)','.-'); 
% title('NN parameters'); 
% xlabel('Time (s)'); 
% lgnd = {'w_1', 'w_2', 'w_3', 'w_4', 'w_5', 'w_6', 'w_7', 'w_8'};
% lgd = legend(lgnd);
% set(lgd, 'Numcolumns', 2);          % Make legend 2 columns
% set(lgd, 'Location', 'Best');       % Put legend in empty spot




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


% Loop data
out_data.loopdata = loopdata;

% Store indices
out_data.inds = inds;

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

% LQ data
out_data.lq_data = [];

% DIRL data
out_data.dirl_data = [];

% Optimal LQ data -- simulation model
out_data.lq_data_opt_sim = lq_data_opt_sim;


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

% Calculate \tilde{u} = u - u_{e} (pre-transformation) 
tu = u - ue;

% Calculate \tilde{u} (post-transformation) 
tup = su * tu;


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

% Calculate integral reinforcement
rxu = 0;
for k = 1:u_sett.numloops

    % Indices of states/outputs corresponding to this loop
    indsyk = loop_cell{k}.indsy;

    % Q_k, R_k
    Qk = loop_cell{k}.Q;
    Rk = loop_cell{k}.R;

    % Get the state vector corresponding to this loop
    [xk, ~, ~] = get_loop_state(x, t, k, xe);

    % State penalty
    xkQkxk = xk' * Qk * xk;

    % Control in this loop
    tupk = tup(indsyk);

    % Control penalty
    ukRkuk = tupk' * Rk * tupk;

    % Add to total
    rxu = rxu + xkQkxk + ukRkuk;

end

% Integral reinforcement
xdot(inds.indsintreinf) = rxu;

        


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

    % Get the state vector corresponding to this loop
    [xk, zk, ek] = get_loop_state(x, t, k, xe);

    % Get rest of state associated with this loop
    txrk = xk(2*nyk+1:nyk+nxk);

    % Get contoller
    Kk = u_sett.K_cell{k}(:,:,1);
    
    % Extract parts of feedback gain matrix K corresponding to each of
    % the states in the partition
    Kz = Kk(:,1:nyk);
    Ky = Kk(:,nyk+1:2*nyk);
    Kr = Kk(:,2*nyk+1:nyk+nxk);

    
    % Calculate control (post-transformation) 
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


% Extract plant states x_p (pre-transformation)
xp = x(inds.indsx);


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
if doall
    kvec = 1:u_sett.numloops;
else
    kvec = k;
end

% Initialize empty output vector
xk = [];
zk = [];
ek = [];

for k = kvec

    % Get indices of state, output vector to pull
    indsxk = loop_cell{k}.indsx;
    indsyk = loop_cell{k}.indsy;
    % indsIntk = loop_cell{k}.indsInt;
    
    % Get number of states, outputs associated with this loop
    nxk = loop_cell{k}.nx;
    nyk = loop_cell{k}.ny;
    
    
    % Pull appropriate state indices
    txpk = txp(indsxk);
    zpk = zp(indsyk);
    tepk = e(indsyk);

    
    % Stack output vector
    % xk = [  zk
    %         txpk ];
    xk = [  xk
            zpk
            tepk
            txpk(nyk+1:end)];

    zk = [  zk
            zpk ];
    ek = [  ek
            tepk ];



end

% If x_3 loop is done, add on h
if doall && u_sett.dox3
    xk = [  xk
            txp(end)];
end




