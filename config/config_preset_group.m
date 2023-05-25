function [preset_list, group_settings] = config_preset_group(preset_group)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PRESET GROUP
%
% Brent Wallace  
%
% 2022-02-16
%
% This program, given a desired preset group, initializes each of the
% presets to run in the group.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% preset_group      (String) Tag corresponding to the preset group to run.
% master_settings   (Struct) Master settings as initialized by main.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% preset_list       ('numpresets' x 1 Cell) The i-th entry of this cell
%                   array is a string which is the tag of the desired
%                   preset to execute as the i-th preset in the group. 
% group_settings    (Struct) This is where any group-shared settings (e.g.,
%                   system, initial conditions, etc.) are stored for
%                   subsequent initialization of each of the presets. All
%                   of these settings are user-configurable, and each are
%                   well-commented below.
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
% PLOT FORMATTING OPTIONS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% % % Plot formatting -- dashed line
% % psett_dash = {'LineStyle', '--'};
% % 
% % % Plot formatting -- colors
% % psett_matlabblue = {'Color', [0 0.4470 0.7410]};
% % psett_matlaborange = {'Color', [0.8500 0.3250 0.0980]};
% % psett_matlabyellow = {'Color', [0.9290 0.6940 0.1250]};
% % psett_matlabpurple = {'Color', [0.4940 0.1840 0.5560]};
% % psett_matlabgreen = {'Color', [0.4660 0.6740 0.1880]};
% % psett_matlablightblue = {'Color', [0.3010 0.7450 0.9330]};
% % psett_matlabmaroon = {'Color', [0.6350 0.0780 0.1840]};



% Degree/radian conversions
D2R =   pi/180;
R2D =   180/pi;


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE PRESET GROUP
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Check if pre-existing preset group settings have been initialized
if isfield(preset_group, 'group_settings')
    group_settings = preset_group.group_settings;
    preset_group = preset_group.tag;
end

% Store the preset group tag in the 'group_settings' struct
group_settings.preset_group = preset_group;

% Master plot settings
psett_master = group_settings.psett_master;

% Plot formatting -- dashed line
psett_dash = psett_master.psett_dash;

% Plot formatting -- colors
psett_matlabblue = psett_master.psett_matlabblue;
psett_matlaborange = psett_master.psett_matlaborange;
psett_matlabyellow = psett_master.psett_matlabyellow;
psett_matlabpurple = psett_master.psett_matlabpurple;
psett_matlabgreen = psett_master.psett_matlabgreen;
psett_matlablightblue = psett_master.psett_matlablightblue;
psett_matlabmaroon = psett_master.psett_matlabmaroon;

switch preset_group


    %%    
    % *********************************************************************
    % *********************************************************************
    %
    % AIRCRAFT EXAMPLES
    %      
    % *********************************************************************
    % *********************************************************************    


    % *********************************************************************
    %
    % NDI HSV -- WANG, STENGEL (2000) -- MAIN PRESET GROUP -- COMPARISON OF
    % 
    % NDI
    % LQ SERVO -- INNER/OUTER
    % FBL
    %
    
    case 'hsv_main'

        % *****************************************************************
        %
        % PRESET GROUP SHARED SETTINGS      
        %        

        % Extract algorithm list
        alg_list = group_settings.alg_list;

        % Extract algorithm names
        algnames = group_settings.algnames;

        % Number of presets executed
        numpresets = size(alg_list,1);

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0vec = zeros(numpresets,1);

        % If model linear, use coupled model (=1) or decoupled (=0)
        c1dc0vec = zeros(numpresets,1);

        % Plant model MP (=1) or NMP (=0)
        Pmp1nmp0vec = zeros(numpresets,1);

        % Algorithm is NDI (=1) or not NDI (=0)
        ndi1notndi0vec = zeros(numpresets,1);

        % Do prefilter (=1) or not (=0)
        pf1nopf0vec = group_settings.pf1nopf0 * ones(numpresets,1);

        % Is training (=1) or not (=0)
        istraining = group_settings.istraining;

        % Do reference command r(t) injection (=1) or not (=0)
        if istraining
            dort = group_settings.dort;
        end

        % Legend, formatting cells, algorithm settings
        preset_list = cell(numpresets,1);
        lgd_p = alg_list;
        indiv_sett_cell = cell(numpresets,1);
        color_sett_cell = cell(numpresets,1);
        for i = 1:numpresets
            switch alg_list{i}
                case algnames.mi_dirl
                    color_sett_cell{i} = psett_matlabblue;
                    indiv_sett_cell{i} = color_sett_cell{i};
%                         indiv_sett_cell{cnt} = psett_dash;
                    lin1nonlin0vec(i) = 0;
%                         lin1nonlin0vec(cnt) = 1;                        
                    Pmp1nmp0vec(i) = 0;
                    ndi1notndi0vec(i) = 0;  
                    preset_list{i} = 'dirl_nonlin';      
                case algnames.si_dirl
                    color_sett_cell{i} = psett_matlabpurple;
                    indiv_sett_cell{i} = color_sett_cell{i};
%                         indiv_sett_cell{cnt} = psett_dash;
                    lin1nonlin0vec(i) = 0;
%                         lin1nonlin0vec(cnt) = 1;                        
                    Pmp1nmp0vec(i) = 0;
                    ndi1notndi0vec(i) = 0;  
                    preset_list{i} = 'dirl_nonlin';                       
                case algnames.lq
                    color_sett_cell{i} = psett_matlabyellow;                    
%                     indiv_sett_cell{i} = ...
%                         [psett_dash; color_sett_cell{i}];
                    indiv_sett_cell{i} = color_sett_cell{i};
                    lin1nonlin0vec(i) = 0;
                    Pmp1nmp0vec(i) = 0;
                    ndi1notndi0vec(i) = 0;  
                preset_list{i} = 'wang_stengel_2000_lq_servo_inout';    
                case algnames.lq_opt_nu
                    color_sett_cell{i} = psett_matlaborange;                    
                    indiv_sett_cell{i} = ...
                        [psett_dash; color_sett_cell{i}];
%                         indiv_sett_cell{i} = {};
                    lin1nonlin0vec(i) = 0;
                    Pmp1nmp0vec(i) = 0;
                    ndi1notndi0vec(i) = 0;  
                preset_list{i} = 'wang_stengel_2000_lq_servo_inout';                    
                case algnames.irl_old
                    color_sett_cell{i} = psett_matlabmaroon;
                    indiv_sett_cell{i} = color_sett_cell{i};
%                         indiv_sett_cell{cnt} = psett_dash;
                    lin1nonlin0vec(i) = 0;
%                         lin1nonlin0vec(cnt) = 1;                        
                    Pmp1nmp0vec(i) = 0;
                    ndi1notndi0vec(i) = 0;  
                    preset_list{i} = 'irl_old';      
                case algnames.mi_irl                  
                    color_sett_cell{i} = psett_matlabgreen;               
                    indiv_sett_cell{i} = color_sett_cell{i};
%                         indiv_sett_cell{cnt} = psett_dash;
                    lin1nonlin0vec(i) = 0;
%                         lin1nonlin0vec(cnt) = 1;                        
                    Pmp1nmp0vec(i) = 0;
                    ndi1notndi0vec(i) = 0;  
                    preset_list{i} = 'dirl_nonlin';  
                case algnames.si_irl                   
                    color_sett_cell{i} = psett_matlablightblue;            
                    indiv_sett_cell{i} = color_sett_cell{i};
%                         indiv_sett_cell{cnt} = psett_dash;
                    lin1nonlin0vec(i) = 0;
%                         lin1nonlin0vec(cnt) = 1;                        
                    Pmp1nmp0vec(i) = 0;
                    ndi1notndi0vec(i) = 0;  
                    preset_list{i} = 'dirl_nonlin';                      
            end
        end



        % ***********************
        %
        % GET SYSTEM DATA
        %   

        % System
        sys = group_settings.sys;
        model = sys.model;
        model_nu = sys.model_nu;

        % Trim
        x_e = model.trimconds.xe_dE;

        % IC
        x0 = x_e;

        % Trim airspeed (ft/s)
        V_e = x_e(model.indV);

        % Trim FPA (rad)
        g_e = x_e(model.indg);

        % Degree/radian conversions
        D2R =   pi/180;
        R2D =   180/pi;

        % ***********************
        %
        % REFERENCE COMMAND SETTINGS
        %
        refcmd = group_settings.refcmd;
        refcmdtype = group_settings.refcmdtype;

        switch refcmd

            case 'training'

                % Simulation, plot time
                tsim = 0;          
                tsim_plot = 0;   

            case 'step_V'

                % Simulation, plot time
                tsim = 200;          
                tsim_plot = 100;    

                % 100 ft/s step-velocity command
                Vr = 100;
                gr = 0;


            case 'step_g'

                % Simulation, plot time
                tsim = 50;          
                tsim_plot = 20;  

                % 1 deg FPA command
                Vr = 0;
                gr = 1*D2R;                

            case 'sin_V'

                % Simulation, plot time
                tsim = 200;          
                tsim_plot = 100;              

                % V(t) = V_{0} + 100 * sin(2pi / T_V * t)
                V_m = V_e;
                A_V = 100;
                T_V = 200;
                g_m = g_e;
                A_g = 0;
                T_g = 1;

            case 'sin_g'                

                % Simulation, plot time
                tsim = 50;          
                tsim_plot = 20;  

                % \gamma(t) = \gamma_{0} + 1 * sin(2pi / T_\gamma * t)
                V_m = V_e;
                A_V = 0;
                T_V = 1;
                g_m = g_e;
                A_g = 1;
                T_g = 25;

            case 'climb'
        
        end

        % ***********************
        %
        % AIRSPEED/FPA REFERENCE COMMANDS
        %

        switch refcmdtype

            case 'step'
      
                % Set params
                V_m = V_e + Vr;
                A_V = 0;
                T_V = 1;
                g_m = g_e + gr;
                A_g = 0;
                T_g = 1;

            case 'multistep'

                % Multi-step reference command
                Vrvec = [0] / 1000;
                Vrtvec = [];
                grvec = [1; 0] * D2R;
                grtvec = [50];
                
                % Set params
                stptoffsetvec = [V_e; g_e];
                stpt_cell = cell(sys.m,1);
                stpt_cell{1} = Vrvec;
                stpt_cell{2} = grvec;
                stptt_cell = cell(sys.m,1);
                stptt_cell{1} = Vrtvec;
                stptt_cell{2} = grtvec;               

      

        end


        % ***********************
        %
        % PLOT SETTINGS
        %

        % Print performance metrics (=1) or don't print (=0)
        print_metrics = 0;

        % Threshold percentages to calculate rise time, settling time (%)
        trpctvec = [90];
        tspctvec = [1; 5; 10];

        % Threshold values to determine settling time
        % Column 1: thresholds for y_1
        % Column 2: thresholds for y_2
        threshmat = [ 1     0.01
                      10    0.1   ];

        % Get variable-name friendly threshold values for storage
        numthresh = size(threshmat,1);
        threshmat_txt = cell(numthresh,2);
        for i = 1:sys.m
            for j = 1:numthresh
                curstr = num2str(threshmat(j,i));
                curstr = strrep(curstr, '.', 'p');
                threshmat_txt{j,i} = curstr;
            end
        end

        % Include legend in plots
        dolegend = 1;        
        
        % Bool which indicates if this preset group is an IC sweep
        is_x0_sweep = 0;


        
        % ***********************
        %
        % STORE SETTINGS
        %

        % Relative path to write data to (on top of relpath)
        group_settings.relpath_data = 'data\';        

        % Methodology has integral augmentation (=1) or not (=0)
        group_settings.has_int_aug = 1;

        % Output variables to track: y = [V, \gamma]^T
        group_settings.inds_xr = [1; 2];

        % Number of presets
        group_settings.numpresets = numpresets;

        % Print performance metrics (=1) or don't print (=0)
        group_settings.print_metrics = print_metrics;

        % Thresholds to determine settling time
        group_settings.trpctvec = trpctvec;
        group_settings.tspctvec = tspctvec;
        group_settings.threshmat = threshmat;      
        group_settings.threshmat_txt = threshmat_txt;

        % Legend entries
        group_settings.lgd_p = lgd_p;

        % Individual plot settings
        group_settings.color_sett_cell = color_sett_cell;        
        group_settings.indiv_sett_cell = indiv_sett_cell;

        group_settings.tsim = tsim;
        group_settings.tsim_plot = tsim_plot;
        group_settings.x0 = x0;

        % Model linear (=1) or nonlinear (=0)
        group_settings.lin1nonlin0vec = lin1nonlin0vec;

        % If model linear, use coupled model (=1) or decoupled (=0)
        group_settings.c1dc0vec = c1dc0vec;

        % Plant model MP (=1) or NMP (=0)
        group_settings.Pmp1nmp0vec = Pmp1nmp0vec;

        % Algorithm is NDI (=1) or not NDI (=0)
        group_settings.ndi1notndi0vec = ndi1notndi0vec;

        % Do prefilter (=1) or not (=0)
        group_settings.pf1nopf0vec = pf1nopf0vec;

        % Commanded airspeed, FPA
        switch refcmdtype
            case 'step'
                r_sett.tag = 'ndi_wang_stengel_2000_sin_Vg';
                r_sett.Vr = Vr;
                r_sett.gr = gr;    
                r_sett.Arvec = [Vr; gr];
            case 'multistep'
                r_sett.tag = 'multistep';
            case 'sin'
                r_sett.tag = 'ndi_wang_stengel_2000_sin_Vg';                
        end
        switch refcmdtype
            case 'training'
                r_sett = [];
            case 'multistep'
                r_sett.stptoffsetvec = stptoffsetvec;
                r_sett.stpt_cell = stpt_cell;
                r_sett.stptt_cell = stptt_cell;
            otherwise                
                r_sett.V_m = V_m;
                r_sett.A_V = A_V;
                r_sett.T_V = T_V;
                r_sett.g_m = g_m;
                r_sett.A_g = A_g;
                r_sett.T_g = T_g;
        end
        group_settings.r_sett = r_sett;

      

        
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        error('*** ERROR: PRESET GROUP TAG NOT RECOGNIZED ***');  
       
end    


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE SYSTEM PARAMETERS AND PLOT SETTINGS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Check if this preset group is executed as part of a group
ismultigroup = isfield(group_settings, 'ismultigroup');

% If not part of a multigroup, initialize the system
if ~ismultigroup

    % Configure settings
    [sys, sys_plot_settings] = config_sys(sys);
    
    % Store system parameters
    group_settings.sys = sys;
    
    % Store plot settings
    group_settings.sys_plot_settings = sys_plot_settings;

end

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE GROUP SETTINGS WHICH ARE ALWAYS DECLARED
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Preset group
group_settings.preset_group = preset_group;

% Do plots for each individual preset in the group
if ~ismultigroup
    group_settings.do_individual_plots = do_individual_plots;
end

% Bool which indicates if this preset group is an IC sweep
group_settings.is_x0_sweep = is_x0_sweep;

% Include legend in plots
group_settings.dolegend = dolegend; 
