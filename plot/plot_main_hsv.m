function plot_main_hsv(alg_settings_cell_master, out_data_cell_master,...
    group_settings_master, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAIN PLOT FUNCTION -- HSV
%
% Brent Wallace  
%
% 2021-12-13
%
% This program, given a specified algorithm and data, plots all the
% relevant data generated by the algorithm and saves the plots.
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% plot_main(alg_settings_cell, out_data_cell, group_settings)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings_cell     (Cell, each entry a Struct) Algorithm 
%                       settings/parameters for subsequent execution
%                       according to desired preset (fields vary by
%                       algorithm, see respective algorithm .m-file for
%                       details).
% out_data_cell         (Cell, each entry a Struct) Output data generated
%                       by the algorithm (fields vary by algorithm, see
%                       respective algorithm .m-file for details).
% group_settings        (Struct) contains preset group settings.
%                       Has the following fields:
%   savefigs            (Boolean) 1 = save figures to PDF. 0 = don't save.
%   relpath             (String) relative file path of folder to save plots
%                       to, if they are to be saved.
%   dolegend            (Boolean) 1 = include preset group legend on plots.
%                       0 = don't include legend.
%   sys_plot_settings   (Struct) contains system plot settings. See
%                       config_sys.m for details.
%   preset_group        (String) Tag of the current preset group being
%                       executed.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% 
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZATION
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

% Relative path to figures -- add timestamp
if savefigs
    timestamp = make_timestamp();
    relpath = [relpath, timestamp];   % Update relative path
    master_settings.relpath = relpath;
end

% Relative path to figures -- add preset-group specific tag
for i = 1:numgroups
    groupnamei = group_settings_master{i}.groupname;
    group_settings_master{i}.relpath = [relpath groupnamei '/'];
end



% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CALL plot_main.m ON EACH PRESET GROUP
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Initialize figure counter
figcount = 1;

% Call plot function
for i = 1:numgroups

    % Pull current group data
    alg_settingsi = alg_settings_cell_master{i};
    out_datai = out_data_cell_master{i};
    group_settingsi = group_settings_master{i};
    group_settingsi.figcount = figcount;

    % Call plot function
    [out_datai, group_settingsi] = ...
        plot_main(alg_settingsi, out_datai, group_settingsi);

    % Update out_data and group_settings for the current group
    out_data_cell_master{i} = out_datai;
    group_settings_master{i} = group_settingsi;
    figcount = group_settingsi.figcount;
end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOT -- CONDITION NUMBER VERSUS ITERATION COUNT
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Update figure counter in master settings
master_settings.figcount = figcount;

% Call function
figcount = plot_cond(alg_settings_cell_master, out_data_cell_master,...
    group_settings_master, master_settings);

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% WRITE TEXTABLES
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

make_tex_table_multigroup(out_data_cell_master,...
    group_settings_master, master_settings)


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SAVE PRESET GROUP DATA TO DIRECTORY
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

data_folder = 'data/';

if savefigs
    
    % Make directory to save data to
    relpath_data = [relpath data_folder];
    mkdir(relpath_data)
    
    % Save data -- alg_settings_cell struct
    varname = 'alg_settings_cell_master';
    save([relpath_data varname], varname);
    
    % Save data -- out_data_cell struct
    varname = 'out_data_cell_master';
    save([relpath_data varname], varname);
    
    % Save data -- group_settings struct
    varname = 'group_settings_master';
    save([relpath_data varname], varname);
    
end

