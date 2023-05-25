function make_tex_table_multigroup(out_data_cell_master,...
    group_settings_master, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% WRITE PERFORMANCE METRIC DATA TO TEXT FILE IN LATEX SYNTAX FOR LATEX
% TABLE
%
% Brent Wallace  
%
% 2022-10-21
%
% This program, given an array of raw performance metric data, tabulates
% the data numerically and writes it to a text file so that the data can be
% copy/pasted into a LaTEX document for use in a table.
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% setts             (Struct) Algorithm settings. Has the following fields:
%   relpath_data    (String) Relative path to write the LaTEX table to
%   filename_data   (String) File name to write the LaTEX table to
%   donewline       (Bool) Insert a LaTEX new line marker '\\' at the end
%                   of each row (=1) or don't insert (=0)
%   dorowpad        (Bool - OPTIONAL) Insert extra custom
%                   padding/formatting terms between each row of the LaTEX
%                   table (=1). If not desired to insert anything between
%                   between rows, leave this field blank. If it is desired,
%                   the following field must be filled out:
%   rowpad_cell     (N x 1 Cell Array - OPTIONAL) Each entry contains the
%                   string to insert in the respective line between each
%                   row of the table, where N is the number of lines of
%                   material desired to insert. Example: If one wanted to
%                   insert a horizontal line in between each row, this
%                   could be
%                       rowpad_cell = {'\hline'}
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% metricdata_cell   ('nummetric' x 'numpresets' Cell Array) Contains the
%                   formatted numerical performance data to be inserted in
%                   the LaTEX table. See 'make_aircraft_textable.m' for
%                   more info.
% metric_cell       ('nummetric' x 1 Cell Array) Each entry is a struct
%                   corresponding to the respective metric data in the row.
%                   Each entry is a struct with the following entries:
%   varname         (String) Programmatic variable name of the metric in
%                   the 'perf_metric_cell' struct.
%   format          (String) Format of how to print the numerical data to
%                   text
%   texname         (String) LaTEX table row name of this metric  
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
% INIT   
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
end

% Do latex newline at end of each row
donewline = 1;



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INDIVIDUAL PRESET TEXTABLES   
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


for i = 1:numgroups

    % Current preset group
    groupi = group_settings_master{i};

    % Check if is of the Wang, Stengel (2000) family
    is_hsv_wang_stengel = groupi.is_hsv_wang_stengel;

    % If is of the Wang, Stengel (2000) family, make textable
    if is_hsv_wang_stengel && savefigs

        % Relative path to performance metric data
        relpath_data = groupi.relpath_data;
    
        % File name of performance metric data
        filename_data = groupi.filename_textable1;
        
        % Define row padding cell struct
        rowpad_cell = groupi.rowpad_cell1;
    
        % Load performance metric data
        tmp = load([relpath_data 'perf_metric_cell.mat']);
        perf_metric_cell = tmp.perf_metric_cell;
    
        % Create textable structs from raw performance metric data
        [~, metricdata_cell, metric_cell] = ...
            make_aircraft_performance_textable(perf_metric_cell);
    
        % Write settings to input struct. See 'make_tex_table.m' for
        % details
        setts.relpath_data = relpath_data;
        setts.filename_data = filename_data;
        setts.donewline = donewline;
        setts.rowpad_cell = rowpad_cell;
    
        % Call LaTEX table function to write data to .txt file
        make_tex_table(metricdata_cell, metric_cell, setts);

    end

end



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% COMBINED PRESET TEXTABLES -- (V, \gamma) RESPONSES ON SAME SYSTEM, SAME
% REFERENCE COMMAND TYPE
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Strings '_V', '_g'
str_V = '_V';
str_g = '_g';

for i = 1:numgroups

    % Current preset group
    groupi = group_settings_master{i};

%     % Get preset group name
%     groupnamei = groupi.groupname;

    % Get simulation system
    model_sim_tagi = groupi.model_sim_tag;

    % Get reference command, reference command type
    refcmdi = groupi.refcmd;
    refcmdtypei = groupi.refcmdtype;

    % Get last two characters of reference command name
%     refcmdi1 = refcmdi(1:end-2);
    refcmdi2 = refcmdi(end-1:end);

    % Check if this was a \gamma response
    isg = strcmp(refcmdi2, str_g);

    % If it was a \gamma response, then check if a V response for the same
    % system and reference command type has been executed
    if isg
        for j = 1:i-1
    
            % Current preset group
            groupj = group_settings_master{j};
        
            % Get preset group name
            groupnamej = groupj.groupname;
        
            % Get simulation system
            model_sim_tagj = groupj.model_sim_tag;
        
            % Get reference command, reference command type
%             refcmdj = groupj.refcmd;
            refcmdtypej = groupj.refcmdtype;  

            % Check if same system
            samesys = strcmp(model_sim_tagi, model_sim_tagj);

            % Check if same reference command type
            sametype = strcmp(refcmdtypei, refcmdtypej);

            % If same system and reference command type, since j < i we
            % know this is a V response. Concatenate textables
            dotable = 0;
            if dotable

                % Relative path to performance metric data
                relpath_datai = groupi.relpath_data; 
                relpath_dataj = groupj.relpath_data;   
            
                % File name of performance metric data
                filename_data = groupi.filename_textable2;                
                               
                % Define row padding cell struct
                rowpad_cell = groupi.rowpad_cell2;
            
                % Load performance metric data
                tmp = load([relpath_datai 'perf_metric_cell.mat']);
                perf_metric_celli = tmp.perf_metric_cell;
                tmp = load([relpath_dataj 'perf_metric_cell.mat']);
                perf_metric_cellj = tmp.perf_metric_cell;

                % Create textable structs from raw performance metric data
                [~, metricdata_celli, metric_cell] = ...
                    make_aircraft_performance_textable(perf_metric_celli);
                [~, metricdata_cellj, ~] = ...
                    make_aircraft_performance_textable(perf_metric_cellj); 

                % Write settings to input struct. See 'make_tex_table.m'
                % for details
                setts.relpath_data = relpath_datai;
                setts.filename_data = filename_data;
                setts.donewline = donewline;
                setts.rowpad_cell = rowpad_cell;

                % Make metric data cell
                metricdata_cell = [metricdata_cellj metricdata_celli];
            
                % Call LaTEX table function to write data to .txt file
                make_tex_table(metricdata_cell, metric_cell, setts);

            end
    
        end
    end

end



