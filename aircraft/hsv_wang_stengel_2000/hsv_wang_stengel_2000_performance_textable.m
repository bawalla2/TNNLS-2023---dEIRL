% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% MAKE PERFORMANCE METRIC DATA LATEX TABLES
%
% Brent Wallace  
%
% 2022-10-21
%
% This program, given an array of raw performance metric data, tabulates
% the data numerically and writes it to a text file so that the data can be
% copy/pasted into a LaTEX document for use in a table. For more details on
% program functionality, see 'make_aircraft_performance_textable.m'
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% setts             (Struct) Algorithm settings. Has the following fields:
%   relpath_data    (String) Relative path to the desired
%                   'perf_metric_cell' struct containing the performance
%                   metric data
%   filename_data   (String) File name of the desired
%                   'perf_metric_cell' struct containing the performance
%                   metric data  
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% metricdatamat     ('nummetric' x 'numpresets' Matrix) Contains the
%                   numerical performance data read from the
%                   'perf_metric_cell' struct, indexed by metric on the
%                   row, by preset on the column
% metricdata_cell   ('nummetric' x 'numpresets' Cell Array) Contains the
%                   same data as 'metricdatamat organized in the same
%                   fashion, but each entry of this cell array is a string
%                   which contains the formatted output ready to be printed
%                   to a LaTEX table
% metricdata_axis   ('nummetric' x 1 Cell Array) Each entry is a string
% with the name of the metric corresponding to the respective row of data
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
%
% LQ SERVO DESIGNS -- STEP VELOCITY COMMAND 
%
% Index ordering is:
%
%   1 = Unfiltered responses
%   2 = Filtered responses
%
% *************************************************************************

% Relative path to performance metric data to read
% relpath_datatmp = ...
%     'aircraft/hsv_wang_stengel_2000/lqr/data/V_h/step_V/100ftps/';
% relpath_datatmp = ...
%     'aircraft/hsv_wang_stengel_2000/lqr/data/V_h/step_h/1000ft/';
% relpath_datatmp = ...
%     'aircraft/hsv_wang_stengel_2000/lqr/data/V_h/step_h/2000ft/';
relpath_datatmp = ...
    '01 data/ES_CL_error/step/';

relpath_data_1 = [relpath_datatmp 'step_V/'];
relpath_data_2 = [relpath_datatmp 'step_g/'];

% File name to write LaTEX table to
filename_data = 'textable.txt';

% Do latex newline at end of each row
donewline = 1;

% Define row padding cell struct
% rowpad_cell = {'\hhline{|-||-|-||-|-||-|-||-|-||-|-|}'};
rowpad_cell = {'\hhline{|-||-|-|-||-|-|-|}'};

% Write settings to input struct. See 'make_tex_table.m' for details
setts.relpath_data = relpath_datatmp;
setts.filename_data = filename_data;
setts.donewline = donewline;
setts.rowpad_cell = rowpad_cell;

% Read preset data and raw simulation data
tmp = load([relpath_data_1 'alg_settings_cell.mat']);
alg_settings_cell_1 = tmp.alg_settings_cell;
tmp = load([relpath_data_1 'out_data_cell.mat']);
out_data_cell_1 = tmp.out_data_cell;
tmp = load([relpath_data_1 'group_settings.mat']);
group_settings_1 = tmp.group_settings;
tmp = load([relpath_data_2 'alg_settings_cell.mat']);
alg_settings_cell_2 = tmp.alg_settings_cell;
tmp = load([relpath_data_2 'out_data_cell.mat']);
out_data_cell_2 = tmp.out_data_cell;
tmp = load([relpath_data_2 'group_settings.mat']);
group_settings_2 = tmp.group_settings;

% Edit save figures control and relative path and to current folder in
% 'group_settings' struct
group_settings_1.savefigs = 1;
group_settings_1.relpath_data = '';
group_settings_2.savefigs = 1;
group_settings_2.relpath_data = '';

% Calculate performance metric data from raw data and write to current
% folder
group_settings_1.relpath = relpath_data_1;
[~] = hsv_wang_stengel_2000_calc...
    (alg_settings_cell_1, out_data_cell_1, group_settings_1);
group_settings_2.relpath = relpath_data_2;
[~] = hsv_wang_stengel_2000_calc...
    (alg_settings_cell_2, out_data_cell_2, group_settings_2);

% Read raw performance metric data
tmp = load([relpath_data_1 'perf_metric_cell.mat']);
perf_metric_cell_1 = tmp.perf_metric_cell;
tmp = load([relpath_data_2 'perf_metric_cell.mat']);
perf_metric_cell_2 = tmp.perf_metric_cell;

% Create textable structs from raw performance metric data
[~, metricdata_cell_1, metric_cell] = ...
    make_aircraft_performance_textable(perf_metric_cell_1);
[~, metricdata_cell_2, ~] = ...
    make_aircraft_performance_textable(perf_metric_cell_2);

% % Read raw textable structs
% tmp = load([relpath_data_1 'perf_metric_cell_table.mat']);
% perf_metric_cell_table_1 = tmp.perf_metric_cell_table;
% tmp = load([relpath_data_2 'perf_metric_cell_table.mat']);
% perf_metric_cell_table_2 = tmp.perf_metric_cell_table;
% 
% % Extract data from raw textable structs
% metricdata_cell_1 = perf_metric_cell_table_1.metricdata_cell;
% metricdata_cell_2 = perf_metric_cell_table_2.metricdata_cell;
% metric_cell = perf_metric_cell_table_1.metric_cell;

% % Alternate data between unfiltered/filtered metric data
% nummetric = size(metric_cell,1);
% numpresets = size(metricdata_cell_1,2);
% metricdata_cell = cell(nummetric,2*numpresets);
% for i = 1:numpresets
%     metricdata_cell(:,2*i-1) = metricdata_cell_1(:,i);
%     metricdata_cell(:,2*i) = metricdata_cell_2(:,i);
% end

% Concatenate data
nummetric = size(metric_cell,1);
numpresets = size(metricdata_cell_1,2);
% metricdata_cell = cell(nummetric,2*numpresets);
metricdata_cell = [metricdata_cell_1 metricdata_cell_2];


% Call LaTEX table function to write data to .txt file
make_tex_table(metricdata_cell, metric_cell, setts);

% Print done
disp('***** DONE WRITING TEXTABLE *****')


