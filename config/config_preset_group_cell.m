function group_settings_master = config_preset_group_cell(master_settings)
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
% This program, given a list of desired preset groups to execute,
% initializes each of the preset groups. Most of the execution in this
% program is configuring automatic settings. However, to change which
% algorithms are executed for a given system, see the variable
% 'alg_list_default'.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% master_settings       (Struct) Master settings as initialized by main.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% group_settings_master     ('numgroups' x 1 Cell) The i-th entry of this   
%                           cell array is itself a struct containing all of
%                           the settings for the corresponding preset
%                           group.
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% UNPACK SETTINGS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Save figures
savefigs = master_settings.savefigs;

% Relative path
relpath = master_settings.relpath;

% Number of preset groups executed
numgroups = master_settings.numgroups;

% Preset group list
preset_group_list = master_settings.preset_group_list; 

% Algorithm names
algnames = master_settings.algnames;

% Master plot settings
psett_master = master_settings.psett_master;

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SETTINGS TAGS, DEFAULT SETTINGS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Algorithm list to execute -- default
alg_list_default = {
                        algnames.lq    
                        algnames.mi_dirl
                        algnames.lq_opt_nu
%                         algnames.fbl
                                };


% Algorithm list to execute -- no FBL
alg_list_no_FBL = {
                        algnames.mi_dirl
                        algnames.lq
                                };

% Algorithm list to execute -- MI-DIRL only
alg_list_midirl_only = { algnames.mi_dirl };
% Algorithm list to execute -- DIRL only
alg_list_dirl_only = { algnames.si_dirl };

% Algorithm list to execute -- new MI-IRL only
alg_list_miirl_only = { algnames.mi_irl };
% Algorithm list to execute -- new IRL only
alg_list_irl_only = { algnames.si_irl };

% Algorithm list to execute -- old IRL only
alg_list_irl_old_only = { algnames.irl_old };



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE PRESET GROUP CELL
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% Number of preset groups executed
group_settings_master = cell(numgroups, 1);


% ***********************
%       
% PRESET GROUP NAME, ALGORITHM NAMES
%     

for i = 1:numgroups

    % Group name
    group_settings_master{i}.groupname = preset_group_list{i};

    % Algorithm names
    group_settings_master{i}.algnames = algnames;

    % Master plot settings
    group_settings_master{i}.psett_master = psett_master;    

end


% ***********************
%       
% DETERMINE IF IS A DIRL TRAINING PRESET GROUP OR NOT
%     

for i = 1:numgroups

    switch preset_group_list{i}

        % ***********************
        %       
        % DIRL TRAINING PRESET GROUPS
        %     
        case[   {'ES_nom_dirl_training'};...
                {'ES_CL_error_dirl_training'};...
                {'ES_nom_dirl_training_no_rt'};...
                {'ES_CL_error_dirl_training_no_rt'};... 
                {'ES_nom_irl_training'};...
                {'ES_CL_error_irl_training'};...                 
                {'ES_nom_irl_training_no_rt'};... 
                {'ES_CL_error_irl_training_no_rt'};... 
                ]
        
            group_settings_master{i}.istraining = 1;
            group_settings_master{i}.dirl1irl0 = 1;
        

        % ***********************
        %       
        % OLD IRL TRAINING PRESET GROUPS
        %     
        case[
                {'ES_nom_irl_old_training'};...
                {'ES_CL_error_irl_old_training'}                
                ]
        
            group_settings_master{i}.istraining = 1;
            group_settings_master{i}.dirl1irl0 = 0;
         

        % ***********************
        %       
        % OTHER PRESET GROUPS
        %          
        otherwise

            group_settings_master{i}.istraining = 0;
            group_settings_master{i}.dirl1irl0 = 1;
    
    end

end
 

% ***********************
%       
% ALGORITHMS EXECUTED, NUMBER OF ALGORITHMS EXECUTED
%     

for i = 1:numgroups

    % If is a training preset, only execute DIRL. Else, execute all
    % algorithms
    if group_settings_master{i}.istraining
        if group_settings_master{i}.dirl1irl0
                switch preset_group_list{i}

                % MI-DIRL training groups
                case[   
                        {'ES_nom_dirl_training'};...
                        {'ES_CL_error_dirl_training'};...
                        ]

                group_settings_master{i}.alg_list = alg_list_midirl_only;

                % DIRL training groups
                case[   
                        {'ES_nom_dirl_training_no_rt'};...
                        {'ES_CL_error_dirl_training_no_rt'};... 
                        ]
        
                group_settings_master{i}.alg_list = alg_list_dirl_only;

                % MI-IRL training groups
                case[
                        {'ES_nom_irl_training'};... 
                        {'ES_CL_error_irl_training'};...                         
                        ]
        
                group_settings_master{i}.alg_list = ...
                    alg_list_miirl_only;  

                % IRL training groups
                case[
                        {'ES_nom_irl_training_no_rt'};... 
                        {'ES_CL_error_irl_training_no_rt'};...                        
                        ]
        
                group_settings_master{i}.alg_list = alg_list_irl_only;   

                end
        else
            % Old IRL
            group_settings_master{i}.alg_list = alg_list_irl_old_only;
        end
    else
        group_settings_master{i}.alg_list = alg_list_default;
    end

    % Number of presets executed
    group_settings_master{i}.numpresets =...
    size(group_settings_master{i}.alg_list,1);

end


% ***********************
%       
% CONFIGURE NOMINAL, SIMULATION MODEL, SET SYSTEM
%    

for i = 1:numgroups

    % Store system parameters
    group_settings_master{i}.sys = master_settings.sys;
    
    % Store plot settings
    group_settings_master{i}.sys_plot_settings =...
        master_settings.sys_plot_settings;

    switch preset_group_list{i}

        % ***********************
        %       
        % NOMINAL MODEL
        %     
        case[   {'ES_nom_dirl_training'};...
                {'ES_nom_irl_training_no_rt'};...
                {'ES_nom_irl_training'};...                
                {'ES_nom_dirl_training_no_rt'};...
                {'ES_nom_irl_old_training'};...
                {'ES_nom_step_V'}; {'ES_nom_step_g'};...
                {'ES_nom_sin_V'}; {'ES_nom_sin_g'}; {'ES_nom_climb'}]
        
            % Nominal model
            model_nom_tag = 'default';
%             model_nom_tag = 'perturbed';        
    
            % Simulation model
            model_sim_tag = 'default';
%             model_sim_tag = 'perturbed';
        

        % ***********************
        %       
        % PERTURBED MODEL
        %          
        case[   {'ES_CL_error_dirl_training'};...
                {'ES_CL_error_irl_training_no_rt'};...   
                {'ES_CL_error_irl_training'};...                  
                {'ES_CL_error_dirl_training_no_rt'};...
                {'ES_CL_error_irl_old_training'};...
                {'ES_CL_error_step_V'}; {'ES_CL_error_step_g'};...
                {'ES_CL_error_sin_V'}; {'ES_CL_error_sin_g'}; ...
                {'ES_CL_error_climb'}]

            % Nominal model
            model_nom_tag = 'default';
%             model_nom_tag = 'perturbed';        
    
            % Simulation model
%             model_sim_tag = 'default';
            model_sim_tag = 'perturbed';
    
    end

    % Set value in the struct
    group_settings_master{i}.model_nom_tag = model_nom_tag;
    group_settings_master{i}.model_sim_tag = model_sim_tag;

end


% ***********************
%       
% CONTROLLER SETTINGS
% 

for i = 1:numgroups

    group_settings_master{i}.lq_data = master_settings.lq_data;
    group_settings_master{i}.lq_data_nu = master_settings.lq_data_nu;
    group_settings_master{i}.pfavec = master_settings.pfavec;  

end


% ***********************
%       
% REFERENCE COMMAND SETTINGS
% 

for i = 1:numgroups

    % ***********************
    %       
    % DIRL TRAINING PRESET GROUPS
    %     
    if group_settings_master{i}.istraining

        % Reference command tag
        group_settings_master{i}.refcmd = 'training';

        % Reference command type
        group_settings_master{i}.refcmdtype = 'training';
    
    % ***********************
    %       
    % NON-DIRL TRAINING PRESET GROUPS
    %             
    else

    switch preset_group_list{i}

        % ***********************
        %       
        % STEP V PRESET GROUPS
        %          
        case[{'ES_nom_step_V'};{'ES_CL_error_step_V'}]

            % Reference command tag
            group_settings_master{i}.refcmd = 'step_V';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'step';            

        % ***********************
        %       
        % STEP \gamma PRESET GROUPS
        %          
        case[{'ES_nom_step_g'};{'ES_CL_error_step_g'}]

            % Reference command tag
            group_settings_master{i}.refcmd = 'step_g';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'step';            
            

        % ***********************
        %       
        % SIN V PRESET GROUPS
        %          
        case[{'ES_nom_sin_V'};{'ES_CL_error_sin_V'}]

            % Reference command tag
            group_settings_master{i}.refcmd = 'sin_V';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'sin';            
            

        % ***********************
        %       
        % SIN \gamma PRESET GROUPS
        %          
        case[{'ES_nom_sin_g'};{'ES_CL_error_sin_g'}]

            % Reference command tag
            group_settings_master{i}.refcmd = 'sin_g';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'sin';   

        % ***********************
        %       
        % CLIMBING MANEUVER PRESET GROUPS
        %        
        case[{'ES_nom_climb'};{'ES_CL_error_climb'}]

            % Reference command tag
            group_settings_master{i}.refcmd = 'climb';

            % Reference command type
            group_settings_master{i}.refcmdtype = 'climb';               

    end       

    end
                
end

% ***********************
%       
% DO PREFILTER OR NOT
% 

for i = 1:numgroups

    switch group_settings_master{i}.refcmdtype

        % ***********************
        %       
        % TRAINING PRESET GROUPS
        %     
        case 'training'
        
            group_settings_master{i}.pf1nopf0 = 0;
        

        % ***********************
        %       
        % STEP PRESET GROUPS
        %          
        case 'step'

            group_settings_master{i}.pf1nopf0 = 1;          

        % ***********************
        %       
        % SIN PRESET GROUPS
        %          
        case 'sin'

            group_settings_master{i}.pf1nopf0 = 0;          
            
        % ***********************
        %       
        % CLIMBING MANEUVER PRESET GROUPS
        %        
        case 'climb'

            group_settings_master{i}.pf1nopf0 = 0;            

    end

end


% ***********************
%       
% DIRL LEARNING SETTINGS -- WHICH LOOPS TO EXECUTE
% 

for i = 1:numgroups

    % Number of presets
    numpresets = group_settings_master{i}.numpresets;

    % Holds which loops to execute for each preset in the group
    irl_setts_cell = cell(numpresets,1);

    for j = 1:numpresets

        % Determine number of loops to execute
        curralg = group_settings_master{i}.alg_list{j};
        switch curralg
            case [{algnames.mi_dirl};{algnames.si_dirl}]
                numloops = 2;
            otherwise
                numloops = 1;                
        end
        % Set number of loops
        irl_setts.numloops = numloops;

        % Which loops to execute
        irl_setts.doloopvec = ones(numloops,1);

        % Store IRL settings for this preset
        irl_setts_cell{j} = irl_setts;

    end

    % Store IRL settings for this group
    group_settings_master{i}.irl_setts_cell = irl_setts_cell;

end

% ***********************
%       
% DIRL LEARNING SETTINGS
% 

for i = 1:numgroups

    % ***********************
    %       
    % TRAINING PRESET GROUPS
    %     
    if group_settings_master{i}.istraining

        % Do learning (=1) or not (=0)
        group_settings_master{i}.dolearning = 1;
        
        % Do plots for each individual preset in the group
        group_settings_master{i}.do_individual_plots = 1;
    
        % Do post-learning sim (=1) or not (=0)
        group_settings_master{i}.dopostlearning = 0;            
    
        % Is of the HSV Wang, Stengel (2000) family
        group_settings_master{i}.is_hsv_wang_stengel = 0;

    
    % ***********************
    %       
    % NON TRAINING PRESET GROUPS
    %             
    else

        % Do learning (=1) or not (=0)
        group_settings_master{i}.dolearning = 0;

        % Do plots for each individual preset in the group
        group_settings_master{i}.do_individual_plots = 0;

        % Do post-learning sim (=1) or not (=0)
        group_settings_master{i}.dopostlearning = 1;

        % Is of the HSV Wang, Stengel (2000) family
        group_settings_master{i}.is_hsv_wang_stengel = 1;

    end

end


% ***********************
%       
% DIRL LEARNING SETTINGS -- DO REFERENCE r(t) OR NOT
% 

for i = 1:numgroups

    % ***********************
    %       
    % DIRL TRAINING PRESET GROUPS
    %     
    if group_settings_master{i}.istraining

        % Check whether or not to insert reference command    
        switch preset_group_list{i}

            case[   {'ES_nom_dirl_training'};...
                    {'ES_nom_irl_training'};
                    {'ES_CL_error_dirl_training'};...
                    {'ES_CL_error_irl_training'};...
                    ]
            
                group_settings_master{i}.dort = 1;
                
            case[   {'ES_CL_error_irl_training_no_rt'};...
                    {'ES_nom_irl_training_no_rt'};...
                    {'ES_nom_dirl_training_no_rt'};...
                    {'ES_CL_error_dirl_training_no_rt'};...
                    {'ES_nom_irl_old_training'};...
                    {'ES_CL_error_irl_old_training'};...
                    ]
    
                group_settings_master{i}.dort = 0;
    
        end  

    end

end

      
% ***********************
%       
% DIRL DATA RELATIVE PATH AND FILE NAME
%     

% Relative path
relpath_dirl = master_settings.relpath_dirl;
relpath_dirl_nom = master_settings.relpath_dirl_nom;
relpath_dirl_CL_error = master_settings.relpath_dirl_CL_error;
filename_data_dirl = master_settings.filename_data_dirl;

for i = 1:numgroups

    % File name
    group_settings_master{i}.filename_data_dirl = filename_data_dirl;

    switch group_settings_master{i}.model_sim_tag

        % ***********************
        %       
        % NOMINAL MODEL
        %     
        case 'default'
        
            group_settings_master{i}.relpath_data_dirl = ...
                    [relpath_dirl relpath_dirl_nom];
        

        % ***********************
        %       
        % PERTURBED MODEL -- CL MODELIN ERROR
        %          
        case 'perturbed'

            group_settings_master{i}.relpath_data_dirl = ...
                    [relpath_dirl relpath_dirl_CL_error];
    
    end

end


% ***********************
%       
% PLOT SETTINGS
%    

for i = 1:numgroups

    % Save figures
    group_settings_master{i}.savefigs = savefigs;

    % Relative path
    group_settings_master{i}.relpath = relpath;
    
    % Override relative path settings in plot_main.m
    group_settings_master{i}.ismultigroup = 1;

end


% ***********************
%       
% TEXTABLE SETTINGS
%   

% Row padding
colpadlr = '|-|';
colpadr = '-|';
hhline = '\hhline{';
lbrac = '}';

for i = 1:numgroups

    % Row padding -- before adding \hhline{}
    numpresetsi = group_settings_master{i}.numpresets;
    rowpadtmp = [colpadlr repmat(colpadr, 1, numpresetsi-1)];
    rowpad1 = [colpadlr rowpadtmp];
    rowpad2 = [rowpad1 rowpadtmp];

    % Row padding -- adding \hhline{}
    rowpad1 = [hhline rowpad1 lbrac];
    rowpad2 = [hhline rowpad2 lbrac];

    group_settings_master{i}.rowpad_cell1 = {rowpad1};
    group_settings_master{i}.rowpad_cell2 = {rowpad2};

    % File names to save .txt files to
    group_settings_master{i}.filename_textable1 = 'textable.txt';
    group_settings_master{i}.filename_textable2 = 'textable_combined.txt';

end






