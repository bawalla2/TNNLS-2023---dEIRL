% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% DECENTRALIZED EXCITABLE INTEGRAL REINFORCEMENT LEARNING (dEIRL) MAIN
% PROGRAM
%
% Brent Wallace  
%
% 2022-07-30
%
% Main file for dEIRL algorithm. Considers infinite-horizon, input-affine
% nonlinear systems.
%
% Associated with work:
%
% B. A. Wallace, J. Si, "Physics-Based Integral Reinforcement Learning: New
% Control Design Algorithms with Theoretical Insights and Performance
% Guarantees" TNNLS 2023.
%
%
% ***** PROGRAM EXECUTION FLOW:
%
% * Select preset groups to execute (see variable 'preset_group_list')
%   below
% * Configure relative paths to data
% * Configure master settings
%   * Configure algorithm settings
%   * Configure master plot formatting
%   * Configure frequency response plot settings
%   * System initialization
%       * Modeling error parameter values \nu
%       * Configure misc system settings -- config_sys.m
%   * Configure controllers for selected system
%   * Configure individual preset group settings 
%       -- config_preset_group_cell.m  
%       -- config_preset_group.m 
% * For each preset in each preset group selected:
%   * Configure individual preset's alg hyperparams -- config_preset.m
%   * Run preset's algorithm  -- programs in 'algs' folder
% * Plot/save figures -- plot_main_hsv.m
%   * For each preset group executed, generate plots -- plot_main.m
%       * State trajectory, control signal plots -- plot_x_u.m
%       * Algorithm-specific plots -- programs in 'plot' folder
%   * Save alg data to directory "00 figures/...../data/"
%   
% ***** DATA MANAGEMENT:
%
% Figures created by this program are written to the folder "00 figures/"
% in a time-stamped subfolder. Raw program data is written to the same
% location. The program data falls under the following three structures:
% 
%   group_settings_master   (Cell array) Each entry contains an
%                           'group_settings' struct for the respective
%                           preset group (see description below)
%   alg_settings_cell_master  (Cell array) Each entry contains a cell array
%                           of 'alg_settings_cell' objects containing the
%                           individual preset settings for each of the
%                           presets executed for the respective preset
%                           group (see description below)
%   out_data_cell_master    (Cell array) Each entry contains an
%                           'out_data_cell' struct for the respective
%                           preset group (see description below)
%
% These three objects are each cell arrays, with the number of entries
% corresponding to the number of preset groups executed. Each entry of
% these cell arrays contains data for the respective preset group of the
% form.
%
%   group_settings      (Struct) Contains shared settings common to all
%                       presets (e.g., state penalty matrix Q). In the case
%                       of an x_0 sweep, this will contain the ICs chosen.
%                       This struct is mainly initialized in the program
%                       config_preset_group.m
%   alg_settings_cell   ('numpresets' x 1 cell) Cell array containing the
%                       algorithm settings of each of the presets in the
%                       group. This cell array is mainly initialized in the
%                       program config_preset.m. Each entry's parameters
%                       are algorithm-specific.
%                       descriptions of each algorithm's hyperparameters
%   out_data_cell       ('numpresets' x 1 cell) After each preset is run,
%                       its algorithm output data (e.g., state trajectory
%                       x(t), NN weight responses, etc.) are stored in this
%                       cell array. The data stored is algorithm-specific.
%
% Data from each individual preset group is also saved under a subfolder
% with the preset group's name
%
%
% ***** PLOTTING PREVIOUSLY-GENERATED DATA:
%
% The program does not need to re-run all of the algorithms to generate
% their data and generate plots again. Given data saved in pre-generated
% 'group_settings_master', 'alg_settings_cell_master', and
% 'out_data_cell_master' structs, the function re_plot_hsv.m may be used
% to plot the pre-existing data. See re_plot_hsv.m for details.
%
% ***** PLOT FORMATTING:
%
% The formatting used for the plots is set programattically by the program
% plot_format.m. The procedure followed to generate plots usually goes as
% follows:
%   Generate plot (title, axes labels, data, etc.) 
%   Specify the figure count in the 'figcount' variable.
%   Format the plot via the following call:
%           p_sett.figcount = figcount;
%           plot_format(p_sett, group_settings);
%       Note: Default formatting options can be over-written for an
%       individual plot by adding additional specs to the 'p_sett' struct
%       (see description of plot_format.m for how to do this).
%   
%
% ***** GENERAL TIPS:
%
% * NOTE: In order to execute properly, the programs must be kept in the
% same file path locations relative to main.m.
% * Set 'savefigs' = 1 (below) to save figures and algorithm data. Data
% gets saved to the relative path specified by the 'relpath' variable
% * To change preset group settings, go to config_preset_group.m
% * Generally speaking, shared hyperparameter values are written in
% config_preset_group.m (e.g., ICs, reference command settings, etc.).
% Specific algorithm learning hyperparameters are written in
% config_preset.m
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIG
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% ***********************
%
% CLEAR VARIABLES, COMMAND WINDOW, FIGURES
%
clear
clc
close all

% ***********************
%
% FIGURES
%
savefigs = 0;               % Save figures control
relpath = '00 figures/';  	% Relative file path for saved figures

% ***********************
%
% SETTINGS
%

% Initialize HSV model (=1) or load previous model (=0)
% init1_load0_hsv = 1;
init1_load0_hsv = 0;

% Initialize LQ servo controller (=1) or load previous controller (=0)
% init1_load0_lq = 1;
% init1_load0_lq = 0;
init1_load0_lq = init1_load0_hsv;


% ***********************
%
% RELATIVE PATHS TO DIRL DATA
%
relpath_dirl = '01 data/DIRL/2022-12-13/';
relpath_dirl_nom = 'nom/';
relpath_dirl_CL_error = 'CL_error/';
filename_data_dirl = 'dirl_data';


% ***********************
%
% INCLUDE UTILITY FOLDER
%
addpath('util');
addpath('util/DIRL');

% ***********************
%
% INCLUDE AIRCRAFT UTILITY FOLDERS
%
addpath('aircraft/util');
addpath('aircraft/hsv_wang_stengel_2000');

% ***********************
%
% INCLUDE CONFIG FUNCTIONS FOLDER
%
addpath('config');

% ***********************
%
% INCLUDE EVALUATION FUNCTIONS FOLDER
%
addpath('eval_functs');

% ***********************
%
% INCLUDE ALGORITHM FOLDER
%
addpath('algs');

% ***********************
%
% INCLUDE PLOT FUNCTIONS FOLDER
%
addpath('plot');

% ***********************
%
% MISC
%

% Suppress warnings pertaining to axes text interpreter
warning('off', 'MATLAB:handle_graphics:exceptions:SceneNode');

% *************************************************************************
%
% METHOD/SYSTEM/DESIGN PRESET GROUPS
%
% These tags correspond to the group of presets to be executed. Each preset
% within the group contains the specific
%
%   Algorithm/methodology (e.g., EIRL, dEIRL, etc.)
%   System (e.g., HSV)
%   Design (with numerical design parameters)
%
% Uncomment the preset groups to execute them. The preset groups will be
% executed in the order they apper in 'preset_group_list'.
%
% *************************************************************************


preset_group = 'hsv_main';

preset_group_list = {   
                        'ES_nom_irl_old_training'
                        'ES_nom_irl_training_no_rt'
                        'ES_nom_irl_training'
                        'ES_nom_dirl_training_no_rt'                        
                        'ES_nom_dirl_training'      
%                         'ES_CL_error_dirl_training'                        
%                         'ES_nom_step_V'
%                         'ES_nom_step_g'
%                         'ES_CL_error_step_V'
%                         'ES_CL_error_step_g'
                                };

% Number of preset groups executed
numgroups = size(preset_group_list, 1);


% *************************************************************************
% *************************************************************************
%
% INITIALIZE/LOAD HSV MODEL
%
% *************************************************************************
% *************************************************************************

% Perturbation parms
% Lift coefficitient modeling error
% nu.CL = 1;
% nu.CL = 0.9;
nu.CL = 0.75;

% Initialize model if desired
if init1_load0_hsv
    hsv_wang_stengel_2000_init(nu);
end

% Load model parameters, aero functions
% Cf. 'hsv_wang_stengel_2000_init.m' for details
relpath_data_hsv = 'aircraft/hsv_wang_stengel_2000/data/';  
models = load([relpath_data_hsv 'hsv_wang_stengel_2000_model.mat']);
models = models.model_struct;
model = models.model; 
model_nu = models.model_nu;

% Configure system settings
sys.tag = 'hsv_wang_stengel_2000';
[sys, sys_plot_settings] = config_sys(sys);
sys.model = model;        
sys.model_nu = model_nu;

% Store system parameters
master_settings.sys = sys;

% Store plot settings
master_settings.sys_plot_settings = sys_plot_settings;

% *************************************************************************
% *************************************************************************
%
% INITIALIZE/LOAD CONTROLLERS
%
% *************************************************************************
% *************************************************************************

% Store relative paths to DIRL data in master settings
master_settings.relpath_dirl = relpath_dirl;
master_settings.relpath_dirl_nom = relpath_dirl_nom;
master_settings.relpath_dirl_CL_error = relpath_dirl_CL_error;
master_settings.filename_data_dirl = filename_data_dirl;

% Controller initialization controls
master_settings.init1_load0_lq = init1_load0_lq;

% Initalize/load controllers
master_settings = config_controllers(master_settings);


%%
% *************************************************************************
% *************************************************************************
%
% CONFIGURE MASTER SETTINGS AND SELECTED PRESET GROUPS
%
% *************************************************************************
% *************************************************************************

% Save figures
master_settings.savefigs = savefigs;

% Relative path
master_settings.relpath = relpath;

% Number of preset groups executed
master_settings.numgroups = numgroups;

% Preset group list
master_settings.preset_group_list = preset_group_list; 


% Algorithm names
algnames.irl_old = 'IRL (old)';
algnames.si_irl = 'SI-EIRL';
algnames.mi_irl = 'EIRL';
algnames.si_dirl = 'SI-dEIRL';
algnames.mi_dirl = 'dEIRL';
algnames.lq = 'Nom. LQ';
algnames.lq_opt_nu = 'Opt. LQ';
master_settings.algnames = algnames;

% ***********************
%
% PLOT FORMATTING
%  

% Plot formatting -- dashed line
psett_master.psett_dash = {'LineStyle', '--'};

% Plot formatting -- colors
psett_master.psett_matlabblue = {'Color', [0 0.4470 0.7410]};
psett_master.psett_matlaborange = {'Color', [0.8500 0.3250 0.0980]};
psett_master.psett_matlabyellow = {'Color', [0.9290 0.6940 0.1250]};
psett_master.psett_matlabpurple = {'Color', [0.4940 0.1840 0.5560]};
psett_master.psett_matlabgreen = {'Color', [0.4660 0.6740 0.1880]};
psett_master.psett_matlablightblue = {'Color', [0.3010 0.7450 0.9330]};
psett_master.psett_matlabmaroon = {'Color', [0.6350 0.0780 0.1840]};

master_settings.psett_master = psett_master;

% ***********************
%
% FREQUENCY RESPONSE PLOT SETTINGS
%  

% Frequency points for SV plots
wmin = -3;
wmax = 2;
nwpts = 500;
wvec = logspace(wmin,wmax,nwpts);
master_settings.wvec = wvec;


% Preset group cell
group_settings_master = ...
    config_preset_group_cell(master_settings);

% Each entry contains the presets executed for the respective group
preset_list_cell = cell(numgroups, 1);

% Initialize 
for i = 1:numgroups

    preset_groupi.group_settings = group_settings_master{i};
    preset_groupi.tag = preset_group;

    [preset_list_cell{i}, group_settings_master{i}] = ...
                config_preset_group(preset_groupi);

end

% Get total number of presets executed
numpresets_tot = 0;
for i = 1:numgroups
    numpresets_tot = numpresets_tot + group_settings_master{i}.numpresets;
end
master_settings.numpresets_tot = numpresets_tot;

    


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAIN LOOP
%
% For each preset in the preset group:
%
%   Run preset configuration to initialize design parameters
%   Run respective preset algorithm and collect algorithm output data 
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Holds algorithm settings for each preset in each group
alg_settings_cell_master = cell(numgroups, 1);

% Holds algorithm output data for each preset in each group
out_data_cell_master = cell(numgroups, 1);

% Cumulative preset counter
cumpresetcnt = 1;


for i = 1:numgroups


    % ***********************
    %
    % PULL CURRENT PRESET GROUP
    %  
    
    % Preset list
    preset_listi = preset_list_cell{i};
    
    % Group settings
    group_settingsi = group_settings_master{i};
    
    % Current preset group name
    preset_group_namei = preset_group_list{i};

    % Display preset group count
    disp('***************************************************************')
    disp('*')
    disp(['* EXECUTING PRESET GROUP   ' num2str(i)...
            '   OUT OF      ' num2str(numgroups)])
    disp(['* CURRENT PRESET GROUP:   ' preset_group_namei] )
    disp('*')
    disp('***************************************************************')


    % ***********************
    %
    % INITIALIZATION
    %  
    
    % Number of presets to execute in the group
    numpresetsi = size(preset_listi, 1);
    
    % ***********************
    %
    % STORAGE
    %  
    
    % Holds algorithm settings for each preset in the group
    alg_settings_celli = cell(numpresetsi, 1);
    
    % Holds algorithm output data for each preset in the group
    out_data_celli = cell(numpresetsi, 1);
    
    for j = 1:numpresetsi
        
        % Display algorithm count
        disp('************************')
        disp('*')
        disp(['* EXECUTING PRESET    ' num2str(cumpresetcnt)...
                '   OUT OF      ' num2str(numpresets_tot)])
        disp('*')
        disp('************************')
        
        % Store the current preset count in group_settings, in case it is
        % needed in the config
        group_settingsi.presetcount = j;
        
        % *********************************************************************
        %
        % SELECT ALGORITHM, SYSTEM, DESIGN PARAMETERS BASED ON PRESET
        % 
        
        alg_settings = config_preset(preset_listi{j}, group_settingsi);
        
        % *********************************************************************
        %
        % RUN ALGORITHM
        % 
        
        % Get algorithm elapsed time -- start timer
        tic;
     
        out_data = eval(['alg_' alg_settings.alg '(alg_settings)']);
        
        % Get algorithm elapsed time -- stop timer
        out_data.runtime = toc;
    
        % Display run time
        disp(['***** RUN TIME OF PRESET ' num2str(cumpresetcnt) ...
                ':     ' num2str(toc) ' s'])
    
        % *****************************************************************
        %
        % STORAGE -- CURRENT PRESET
        % 
        
        % Store preset settings
        alg_settings_celli{j} = alg_settings;
        
        % Store algorithm output data
        out_data_celli{j} = out_data;
        
        % Increment cumulative preset counter
        cumpresetcnt = cumpresetcnt + 1;
        
    end


    % *****************************************************************
    %
    % STORAGE -- CURRENT PRESET GROUP
    % 
    
    % Store preset settings
    alg_settings_cell_master{i} = alg_settings_celli;
    
    % Store algorithm output data
    out_data_cell_master{i} = out_data_celli;    

end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOT/SAVE FIGURES
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Call plot function
plot_main_hsv(alg_settings_cell_master, out_data_cell_master,...
    group_settings_master, master_settings);



% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
% 
% ********************************************************************** ***
% *************************************************************************
% *************************************************************************

% Display complete
disp('*******************************************************************')
disp('*******************************************************************')
disp('*')
disp(['* MAIN PROGRAM COMPLETE'])
disp('*')
disp('*******************************************************************')
disp('*******************************************************************')

