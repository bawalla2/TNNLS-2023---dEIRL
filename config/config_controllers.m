function master_settings = config_controllers(master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIG LQ SERVO CONTROLLERS
%
% Brent Wallace  
%
% 2021-12-13
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

% *************************************************************************
%
% UNPACK SETTINGS
% 
% *************************************************************************

% System
sys = master_settings.sys;
model = sys.model;
model_nu = sys.model_nu;

% Controller initialization controls
init1_load0_lq = master_settings.init1_load0_lq;

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE CONTROLLERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
%
% LQ SERVO
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% LQ SERVO INNER-OUTER DESIGN PARAMETERS
%
% NOTE: State partitioned as (see below)
%
%      = [  z
%           y
%           x_r ]
%


Qg = diag([1 1 0 0]);
Rg = 0.01;

QV = diag([1 1]);
RV = 15;

% ***********************
%
% GET CONTROLLER -- LQ SERVO INNER/OUTER
%   

% Relative path to controller params
relpath_ctrl_tmp = ...
    'aircraft/hsv_wang_stengel_2000/lq_servo_inout/data/';

% File name to save to
filename_tmp = 'lq_data.mat';
filename_nu_tmp = 'lq_data_nu.mat';
if init1_load0_lq

    % Initialize relevant algorithm settings
    alg_settings.sys = sys;
    alg_settings.model_d = model;
    alg_settings.Qg = Qg;
    alg_settings.Rg = Rg;
    alg_settings.QV = QV;
    alg_settings.RV = RV;
    alg_settings.relpath_data = relpath_ctrl_tmp;
    alg_settings.filename = filename_tmp;
    
    % Initialize controller -- nominal parameters
    hsv_wang_stengel_2000_lq_servo_inout_init(alg_settings);

    % Initialize controller -- perturbed parameters
    alg_settings.model_d = model_nu;
    alg_settings.filename = filename_nu_tmp;
    hsv_wang_stengel_2000_lq_servo_inout_init(alg_settings);

end

% Load controller
data = load([relpath_ctrl_tmp filename_tmp]);
lq_data = data.lq_data;
data = load([relpath_ctrl_tmp filename_nu_tmp]);
lq_data_nu = data.lq_data;       



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE DATA
% 
% *************************************************************************
% *************************************************************************       
% *************************************************************************

% ***********************
%
% LQ SERVO
%   

master_settings.lq_data = lq_data;
master_settings.lq_data_nu = lq_data_nu; 
master_settings.pfavec = lq_data.pfavec;

