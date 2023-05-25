function plot_format(plot_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% APPLY BASIC FORMATTING TO PLOT
%
% Brent Wallace  
%
% 2022-01-13
%
% This program applies basic formatting to a given plot.
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% plot_format(figcount, group_settings)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% plot_settings     (Struct) Structure containing various plot controls.
%                   Has the following fields:
%   figcount        (Integer) Figure number of the figure to edit
%                   formatting of.
%
% group_settings    (Struct) Structure containing the overall preset group
%                   settings. See main.m for description of fields.
%
% ***** TO OVERRIDE DEFAULT SETTINGS
%
% Each of the formatting options available in this program can be found
% under the 'settings_cell' variable (declared below). If it is desired to
% override any of these formatting values, then in the 'plot_settings'
% input struct, declare a field named '.custom_sett' under which to
% override them. To illustrate, here is an example:
%
%   If it is desired to change the default title font size, the
%   corresponding formatting variable is 'ttl_fontsize'. If one wants to
%   change this to, say, 10, they would in plot_settings.custom_sett
%   declare:
%               plot_settings.custom_sett.ttl_fontsize = 10;
%
% NOTE: Do not declare '.custom_sett' if no default settings are to be
% overriden.
%
% ***** TO OVERRIDE DEFAULT SETTINGS -- INDIVIDUAL PLOT ENTRIES
%
% To overrride settings for individual entries within a plot, then in the
% 'plot_settings' input struct, declare a field named '.indiv_sett_cell'.
% If the plot consists of N 'Line' objects (e.g., N = 2, one curve showing
% x_1(t), the other curve showing x_2(t)), then 'indiv_sett_cell' is an Nx1
% cell array. If no settings are to be applied to line object k (1 <= k
% <= N), then indiv_sett_cell{k} can be left empty. Else,
% indiv_sett_cell{k} must be itself a n_k x 2 cell array, where n_k is the
% number of custom settings to apply to line object k. The first entry of
% each row of indiv_sett_cell{k} is the 'Name' entry of the line object to
% edit (e.g., 'LineStyle'). The second entry is the 'Value' entry of the
% line object to edit (e.g., '--' for a dashed line).
%
% EXAMPLE: N = 2. Line 1: x_1(t), line 2: x_2(t). We want to keep the
% formatting for line 1 default, and make line 2 dashed. We would declare
% 'indiv_sett_cell' as:
%
% psett_dashedline = {'LineStyle', '--'};
% indiv_sett_cell = {{}; psett_dashedline};
%
% NOTE: Any custom settings in 'custom_sett' are applied FIRST to ALL line
% objects in the plot, and then any individual plot settings in
% 'indiv_sett_cell' are applied SECOND.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% NONE (Re-formatted plots)
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZATION
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% UNPACK USER SETTINGS
% 
% *************************************************************************

% ***********************
%       
% GET FIGURE SPECIFIED BY USER
%  

% Figure number of figure to be edited
figcount = plot_settings.figcount;

% Get figure
fig = figure(figcount);

% Get CurrentAxes object of figure
ax = fig.CurrentAxes;

% Get x, y, z axis objects
ax_x = ax.XAxis;
ax_y = ax.YAxis;
ax_z = ax.ZAxis;

% Get any line object present in the figure
lines = findobj(fig, 'Type', 'Line');
haslines = ~isempty(lines);
numlines = size(lines,1);

% Get the index of each line object in the series in the figure
% MATLAB sorts the line objects alphabetically once they are named, so the
% order might have changed
indslines = zeros(numlines,1);
if haslines

    for i = 1:numlines

        indslines(i) = lines(i).SeriesIndex;

    end

end

% Get Legend object of figure (is inside the CurrentAxes object)
lgd = ax.Legend;
haslgd = ~isempty(lgd);

% Get if the plot is a 2D plot or 3D plot
twoD1_threeD0 = isequal(ax.View, [0 90]);

% Get x, y, z window limits
ax_x_lim = ax_x.Limits;
ax_y_lim = ax_y.Limits;
ax_z_lim = ax_z.Limits;

% Get x, y, z axis ticks
ax_x_ticks = ax_x.TickValues;
ax_y_ticks = ax_y.TickValues;
ax_z_ticks = ax_z.TickValues;

% ***********************
%       
% CHECK FOR CUSTOM SETTINGS SPECIFIED BY USER
%  

% Any custom settings that have been specified by the user to override the
% defaults
override_sett = isfield(plot_settings, 'custom_sett');
if override_sett   
    custom_sett = plot_settings.custom_sett;
end

% ***********************
%       
% CHECK FOR INDIVIDUAL LINE OBJECT SETTINGS SPECIFIED BY USER
%  

% Any custom settings that have been specified by the user to override the
% defaults
do_indiv_sett = isfield(plot_settings, 'indiv_sett_cell');
if do_indiv_sett   
    indiv_sett_cell = plot_settings.indiv_sett_cell;
end

% *************************************************************************
%
% CELL OF ALL FORMATTING SETTING VARIABLE NAMES
% 
% *************************************************************************

settings_cell = {
                    % Title formatting
                    'ttl_font'
                    'ttl_fontsize'
                    'ttl_fontweight'
                    'ttl_interp'
                    % Axis limits
                    'axlim_x'
                    'axlim_y'
                    'axlim_z'
                    % Axis tick formatting
                    'tick_font'
                    'tick_fontsize'
                    'tick_fontweight'
                    'tick_interp'  
                    'tick_rot'
                    'keep_ticks'
                    % Axis label formatting
                    'lbl_font'
                    'lbl_fontsize'
                    'lbl_fontweight'
                    'lbl_interp' 
                    % Legend formatting
                    'lgd_font'
                    'lgd_fontsize'
                    'lgd_fontweight'
                    'lgd_interp'   
                    'lgd_loc'
                    'lgd_position'
                    % Line formatting
                    'line_width'
                    % Grid formatting
                    'grid_on'
                                    };
                                
len_settings_cell = size(settings_cell, 1);                                

% *************************************************************************
%
% DEFAULT PLOT FORMATTING SETTINGS
% 
% *************************************************************************

% ***********************
%       
% TITLE FORMATTING
%  

% Title font -- MATLAB Default: 'Helvetica'
ttl_font = 'Helvetica';

% Title font size -- MATLAB Default: 11
ttl_fontsize = 15;

% Title font weight -- MATLAB Default: 'bold'
ttl_fontweight = 'bold';

% Title interpreter -- MATLAB Default: 'tex'
ttl_interp = 'latex';

% ***********************
%       
% AXIS LIMITS
%
% NOTE: Axis limit settings are changed only if the user declared overrides
%  

% Flags to set axis limits (low by default, set high if custom setting
% detected)
setxlim = 0;
setylim = 0;
setzlim = 0;


% ***********************
%       
% AXIS TICK FORMATTING
%  

% Tick font -- MATLAB Default: 'Helvetica'
tick_font = 'Helvetica';

% Tick font size -- MATLAB Default: 10
tick_fontsize = 12;

% Tick font weight -- MATLAB Default: 'normal'
tick_fontweight = 'bold';

% Tick interpreter -- MATLAB Default: 'tex'
tick_interp = 'tex';

% Tick label rotation -- MATLAB Default: 0 (vertical)
tick_rot = 0;

% Keep current tick values (=1), or let formatting changes dictate new tick
% values (=0)
keep_ticks = 1;


% ***********************
%       
% AXIS LABEL FORMATTING
%  

% Label font -- MATLAB Default: 'Helvetica'
lbl_font = 'Helvetica';

% Label font size -- MATLAB Default: 11
lbl_fontsize = 20;

% Label font weight -- MATLAB Default: 'normal'
lbl_fontweight = 'bold';

% Label interpreter -- MATLAB Default: 'tex'
lbl_interp = 'latex';


% ***********************
%       
% LEGEND FORMATTING
%  

% Legend font -- MATLAB Default: 'Helvetica'
lgd_font = 'Helvetica';

% Legend font size -- MATLAB Default: 9
lgd_fontsize = 12;

% Legend font weight -- MATLAB Default: 'normal'
lgd_fontweight = 'normal';
% lgd_fontweight = 'bold';

% Legend interpreter -- MATLAB Default: 'tex'
lgd_interp = 'latex';

% Legend location -- MATLAB Default: 'northeast'
if twoD1_threeD0
%     lgd_loc = 'northeast';
    lgd_loc = 'best';
else
    lgd_loc = 'northeast';
%     lgd_loc = 'best';
end
% lgd_loc = 'northeast';

% Legend position flag -- set high if user overrwrote the setting only
setlgdpos = 0;


% ***********************
%       
% LINE FORMATTING
%

% Line width -- MATLAB Default: 0.5
line_width = 2;

% ***********************
%       
% GRID FORMATTING
% 

% Turn grid on or not -- MATLAB Default: 0
grid_on = 1;


% *************************************************************************
%
% OVERRIDE DEFAULT SETTINGS IF USER SPECIFIES
% 
% *************************************************************************

if override_sett

    for i = 1:len_settings_cell
        
        currsett = settings_cell{i};
        
        if isfield(plot_settings.custom_sett, currsett)
            
            switch currsett

                
                % ***********************
                %       
                % TITLE FORMATTING
                % 
                
                case 'ttl_font'
                    
                    ttl_font = custom_sett.ttl_font;                
                
                case 'ttl_fontsize'
                    
                    ttl_fontsize = custom_sett.ttl_fontsize;
                    
                case 'ttl_fontweight'
                    
                    ttl_fontsize = custom_sett.ttl_fontweight;
                        
                case 'ttl_interp'
                    
                    ttl_interp = custom_sett.ttl_interp;   

                % ***********************
                %       
                % AXIS LIMITS
                %     

                case 'axlim_x'

                    setxlim = 1;
                    axlim_x = custom_sett.axlim_x; 

                case 'axlim_y'

                    setylim = 1;
                    axlim_y = custom_sett.axlim_y; 

                case 'axlim_z'

                    setzlim = 1;
                    axlim_z = custom_sett.axlim_z;                     

                % ***********************
                %       
                % AXIS TICK FORMATTING
                %                     
                    
                case 'tick_font'
                    
                    tick_font = custom_sett.tick_font;  
                
                case 'tick_fontsize'
                    
                    tick_fontsize = custom_sett.tick_fontsize;
                    
                case 'tick_fontweight'
                    
                    tick_fontsize = custom_sett.tick_fontweight;
                        
                case 'tick_interp'
                    
                    tick_interp = custom_sett.tick_interp;        
                    
                case 'tick_rot'
                    
                    tick_rot = custom_sett.tick_rot;                     

                case 'keep_ticks'
                    
                    keep_ticks = custom_sett.keep_ticks;                      
                    
                % ***********************
                %       
                % AXIS LABEL FORMATTING
                % 
                
                case 'lbl_font'
                    
                    lbl_font = custom_sett.lbl_font;  
                
                case 'lbl_fontsize'
                    
                    lbl_fontsize = custom_sett.lbl_fontsize;
                    
                case 'lbl_fontweight'
                    
                    lbl_fontsize = custom_sett.lbl_fontweight;
                        
                case 'lbl_interp'
                    
                    lbl_interp = custom_sett.lbl_interp;                       
                    
                % ***********************
                %       
                % LEGEND FORMATTING
                %                     
                    
                case 'lgd_font'
                    
                    lgd_font = custom_sett.lgd_font;                
                
                case 'lgd_fontsize'
                    
                    lgd_fontsize = custom_sett.lgd_fontsize;
                    
                case 'lgd_fontweight'
                    
                    lgd_fontsize = custom_sett.lgd_fontweight;
                        
                case 'lgd_interp'
                    
                    lgd_interp = custom_sett.lgd_interp;
                    
                case 'lgd_loc'
                    
                    lgd_loc = custom_sett.lgd_loc;    

                case 'lgd_position'
                    
                    % Set flag to override legend position
                    setlgdpos = 1;

                    lgd_position = custom_sett.lgd_position;  

                % ***********************
                %       
                % LINE FORMATTING
                %                    

                case 'line_width'
                    
                    line_width = custom_sett.line_width;                 
                
                % ***********************
                %       
                % GRID FORMATTING
                %                     
                    
                case 'grid_on'
                    
                    grid_on = custom_sett.grid_on;                       
                    
                % ***********************
                %
                % THROW ERROR IF TAG DOES NOT COME UP A MATCH
                %   

                otherwise

                    error(['*** ERROR: PLOT SETTING'...
                        ' VARIABLE NAME NOT RECOGNIZED ***']);  
                    
                    
            end
            
        end
        
    end
    
end




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



% ***********************
%       
% TITLE FORMATTING
%  

% Title font
set(ax.Title, 'FontName', ttl_font);

% Title font size
set(ax.Title, 'FontSize', ttl_fontsize);

% Title font weight
set(ax.Title, 'FontWeight', ttl_fontweight);

% Title interpreter
set(ax.Title, 'interpreter', ttl_interp);


% ***********************
%       
% AXIS LIMITS
%     

% x-axis limits
if setxlim
    xlim(axlim_x);
end

% y-axis limits
if setylim
    ylim(axlim_y);
end

% z-axis limits
if setzlim
    zlim(axlim_z);
end

% ***********************
%       
% AXIS TICK FORMATTING
%  

% Tick font
set(ax_x, 'FontName', tick_font);
set(ax_y, 'FontName', tick_font);
set(ax_z, 'FontName', tick_font);

% Tick font size
set(ax_x, 'FontSize', tick_fontsize);
set(ax_y, 'FontSize', tick_fontsize);
set(ax_z, 'FontSize', tick_fontsize);

% Tick font weight
set(ax_x, 'FontWeight', tick_fontweight);
set(ax_y, 'FontWeight', tick_fontweight);
set(ax_z, 'FontWeight', tick_fontweight);

% Tick interpreter
set(ax_x, 'TickLabelInterpreter', tick_interp);
set(ax_y, 'TickLabelInterpreter', tick_interp);
set(ax_z, 'TickLabelInterpreter', tick_interp);

% Tick label rotation
set(ax_x, 'TickLabelRotation', tick_rot);
set(ax_y, 'TickLabelRotation', tick_rot);
set(ax_z, 'TickLabelRotation', tick_rot);

% Return to original axes limits
if ~setxlim
    set(ax_x, 'Limits', ax_x_lim);
end
if ~setylim
    set(ax_y, 'Limits', ax_y_lim);
end
if ~setzlim
    set(ax_z, 'Limits', ax_z_lim);
end

% Keep current tick values (=1), or let formatting changes dictate new tick
% values (=0)
if keep_ticks
    if ~setxlim
        set(ax_x, 'TickValues', ax_x_ticks);
    end
    if ~setylim
        set(ax_y, 'TickValues', ax_y_ticks);
    end
    if ~setzlim
        set(ax_z, 'TickValues', ax_z_ticks);
    end
end


% ***********************
%       
% AXIS LABEL FORMATTING
%  

% Label font
set(ax.XLabel, 'FontName', lbl_font);
set(ax.YLabel, 'FontName', lbl_font);
set(ax.ZLabel, 'FontName', lbl_font);

% Label font size
set(ax.XLabel, 'FontSize', lbl_fontsize);
set(ax.YLabel, 'FontSize', lbl_fontsize);
set(ax.ZLabel, 'FontSize', lbl_fontsize);

% Label font weight
set(ax.XLabel, 'FontWeight', lbl_fontweight);
set(ax.YLabel, 'FontWeight', lbl_fontweight);
set(ax.ZLabel, 'FontWeight', lbl_fontweight);

% Label interpreter
set(ax.XLabel, 'interpreter', lbl_interp);
set(ax.YLabel, 'interpreter', lbl_interp);
set(ax.ZLabel, 'interpreter', lbl_interp);


% ***********************
%       
% LEGEND FORMATTING
%  

if haslgd

    % Legend font
    set(lgd, 'FontName', lgd_font);

    % Legend font size
    set(lgd, 'FontSize', lgd_fontsize);

    % Legend font weight
    set(lgd, 'FontWeight', lgd_fontweight);

    % Legend interpreter
    set(lgd, 'interpreter', lgd_interp);

    % Legend location
    set(lgd, 'location', lgd_loc);

    % Legend position -- manual override only
    if setlgdpos
        set(lgd, 'position', lgd_position);
    end
    

end



% ***********************
%       
% LINE FORMATTING
%  

if haslines
   
    % Line width
    set(lines(:), 'LineWidth', line_width);
    
end

% ***********************
%       
% GRID FORMATTING
%  

% Turn grid on or not
if grid_on
    grid on;
end


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% APPLY INDIVIDUAL LINE OBJECT SETTINGS IF USER SPECIFIED
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

if do_indiv_sett

    for i = 1:numlines

        % Extract the settings cell for the current line object
        currsetti = indiv_sett_cell{i};

        % Check how many (if any) settings are to be changed for this line
        % object
        numsetti = size(currsetti,1);

        % If settings are to be changed, get current line object
        if numsetti > 0

            currline = lines(indslines(i));

        end

        % Edit settings
        for j = 1:numsetti

            currsettij = currsetti(j,:);
            set(currline, currsettij{1}, currsettij{2});

        end

    end

end
