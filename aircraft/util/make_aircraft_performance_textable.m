function [metricdatatmat, metricdata_cell, metric_cell] = ...
    make_aircraft_performance_textable(perf_metric_cell)
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
% perf_metric_cell  ('numpresets' x 1 Cell Array) Each entry contains the
%                   performance metric data of the respective preset in the
%                   list
%
% ---------------
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
% metric_cell       ('nummetric' x 1 Cell Array) Each entry is a struct
%                   corresponding to the respective metric data in the row.
%                   Each entry is a struct with the following entries:
%   varname         (String) Programmatic variable name of the metric in
%                   the 'perf_metric_cell' struct.
%   format          (String) Format of how to print the numerical data to
%                   text
%   texname         (String) LaTEX table row name of this metric      
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
%
% INIT   
%
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% EXTRACT PROGRAM SETTINGS   
%
% *************************************************************************

% % Relative path to performance metric data
% relpath_data = setts.relpath_data;
% 
% % File name of performance metric data
% filename_data = setts.filename_data;
% 
% % Check if latex format padding is to be added between successive rows of
% % data
% dorowpad = isfield(setts, 'rowpad_cell');
% 
% % If row padding is to be performed, extract the cell array of padding
% % entries
% if dorowpad
%     rowpad_cell = setts.rowpad_cell;
% end

% Number of presets executed
numpresets = size(perf_metric_cell,1);

% Control -- do custom threshold settling time metrics (=1) or not (=0)
do_ts_cust = 0;

% Control -- NDI-specific metrics (=1) or not (=0)
do_ndi = 0;

% Check if this was a step response
isstep = isfield(perf_metric_cell{1}, 'ts_eV1pct');


% *************************************************************************
%
%
% TRACKING/STEP RESPONSE METRICS
%
% *************************************************************************

% Dummy temp performance metric struct to get data from
pfmtmp = perf_metric_cell{1};

% Output variable indices
inds_xr = pfmtmp.inds_xr; 

% Output properties
y_propts_cell = pfmtmp.y_propts_cell;

% Settling, rise time thresholds
isstep = pfmtmp.isstep;
if isstep
    trpctvec = pfmtmp.trpctvec;
    tspctvec = pfmtmp.tspctvec;
end
threshmat = pfmtmp.threshmat;
threshmat_txt = pfmtmp.threshmat_txt;

% Number of threshold values to check, number of inputs
numthresh = size(threshmat,1);
m = size(threshmat,2);

% *************************************************************************
%
% FORMATTING SETTINGS 
%
% *************************************************************************

% % Default output format -- 4 significant digits, exponential or fixed-point
% % (whichever is shorter)
% format_default = '%11.4g';

% Default output format 
format_default = '%5.4g';

% Format -- exponential notation
format_exp = '%.2e';

%%
% *************************************************************************
% *************************************************************************
%
% MASTER LIST OF PERFORMANCE METRICS
%
% Each of these cell arrays has the following entries:
%
% varname           (String) Programmatic variable name of the metric in
%                   the 'perf_metric_cell' struct.
% format            (String) Format of how to print the numerical data to
%                   text
% texname           (String) LaTEX table row name of this metric
%
% *************************************************************************
% *************************************************************************

% % Begin counter of number of metrics
% nummetric = 0;
% 

% Initialize empty metric cell array
metric_cell = [];



% *************************************************************************
%
% TRACKING/STEP RESPONSE METRICS
%
% *************************************************************************

for i = 1:m

    % ***********************
    %
    % GET CURRENT OUTPUT VARIABLE
    %
    
    ind_xri = inds_xr(i);

    y_propts = y_propts_cell{ind_xri};

    currvarname = y_propts.varname;
    currtexname = y_propts.texname;

    % Threshold vector
    threshvec = threshmat(:,i);
    threshvec_txt = threshmat_txt(:,i);


    % ***********************
    %
    % e_{y_i,max}
    %
    
    metr.varname = ['e' currvarname 'max'];
    metr.texname = ['$e_{' currtexname ',max}$'];
    metr.format = format_default;

    % Add to metric cell array
    metric_cell = [metric_cell ; {metr}];
    
    % ***********************
    %
    % t_{s, e_{y_i}, THRESHOLD}
    %
    
    for j = 1:numthresh
        % Current value to determine settling time to
        currsetval = threshvec(j);
        
        metr.varname = ['ts_' currvarname '_' threshvec_txt{j}];
        metr.texname = ['$t_{s, ' currtexname ...
            ', ' num2str(currsetval) '}$'];
        metr.format = format_default;

        % Add to metric cell array
        metric_cell = [metric_cell ; {metr}];
    end

    % ***********************
    %
    % t_{s, e_{y_i}, %}
    %
    
    if isstep
    for j = 1:length(tspctvec)
        % Current % value to determine settling time to (%)
        currpct = tspctvec(j);
 
        metr.varname = ['ts_' currvarname '_' num2str(currpct) 'pct'];
        metr.texname = ['$t_{s, ' currtexname ...
            ', ' num2str(currpct) ' \%}$'];
        metr.format = format_default;

        % Add to metric cell array
        metric_cell = [metric_cell ; {metr}];

    end
    end
    

    % ***********************
    %
    % t_{r, y_i, %}
    %
    
    if isstep
    for j = 1:length(trpctvec)
        % Current % value to determine rise time to (%)
        currpct = trpctvec(j);
        
        metr.varname = ['tr_' currvarname '_' num2str(currpct) 'pct'];
        metr.texname = ['$t_{r, ' currtexname ...
            ', ' num2str(currpct) ' \%}$'];
        metr.format = format_default;

        % Add to metric cell array
        metric_cell = [metric_cell ; {metr}];
    end
    end
    
    % ***********************
    %
    % M_{p, y_i}
    %
    
    if isstep
    metr.varname = ['Mp' currvarname];
    metr.texname = ['$M_{p, ' currtexname '}$'];
    metr.format = format_default;

    % Add to metric cell array
    metric_cell = [metric_cell ; {metr}];         
    end

       

end

% *************************************************************************
%
% OTHER METRICS
%
% *************************************************************************


% ***********************
%
% V_min, V_max
%

metr_Vmin.varname = 'Vmin';
metr_Vmin.texname = '$V_{min}$';
metr_Vmin.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_Vmin}];

metr_Vmax.varname = 'Vmax';
metr_Vmax.texname = '$V_{max}$';
metr_Vmax.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_Vmax}];

% ***********************
%
% \gamma_min, \gamma_max
%

metr_gammamin.varname = 'gammamin';
metr_gammamin.texname = '$\gamma_{min}$';
metr_gammamin.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_gammamin}];

metr_gammamax.varname = 'gammamax';
metr_gammamax.texname = '$\gamma_{max}$';
metr_gammamax.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_gammamax}];

% ***********************
%
% h_min, h_max
%

metr_hmin.varname = 'hmin';
metr_hmin.texname = '$h_{min}$';
metr_hmin.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_hmin}];

metr_hmax.varname = 'hmax';
metr_hmax.texname = '$h_{max}$';
metr_hmax.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_hmax}];

% ***********************
%
% \alpha_min, \alpha_max
%

metr_alphamin.varname = 'alphamin';
metr_alphamin.texname = '$\alpha_{min}$';
metr_alphamin.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_alphamin}];

metr_alphamax.varname = 'alphamax';
metr_alphamax.texname = '$\alpha_{max}$';
metr_alphamax.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_alphamax}];


% ***********************
%
% \d_Tmin, \d_Tmax
%

metr_dTmin.varname = 'dTmin';
metr_dTmin.texname = '$\delta_{T,min}$';
metr_dTmin.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_dTmin}];

metr_dTmax.varname = 'dTmax';
metr_dTmax.texname = '$\delta_{T,max}$';
metr_dTmax.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_dTmax}];


% ***********************
%
% T_min, T_max
%

metr_Tmin.varname = 'Tmin';
metr_Tmin.texname = '$T_{min}$';
% metr_Tmin.format = format_default;
metr_Tmin.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_Tmin}];

metr_Tmax.varname = 'Tmax';
metr_Tmax.texname = '$T_{max}$';
% metr_Tmax.format = format_default;
metr_Tmax.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_Tmax}];

% ***********************
%
% \d_Emin, \d_Emax
%

metr_dEmin.varname = 'dEmin';
metr_dEmin.texname = '$\delta_{E,min}$';
metr_dEmin.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_dEmin}];

metr_dEmax.varname = 'dEmax';
metr_dEmax.texname = '$\delta_{E,max}$';
metr_dEmax.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_dEmax}];

% ***********************
%
% q_{\infty,min}, q_{\infty,max}
%

metr_qinfmin.varname = 'qinfmin';
metr_qinfmin.texname = '$\overline{q}_{min}$';
metr_qinfmin.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_qinfmin}];

metr_qinfmax.varname = 'qinfmax';
metr_qinfmax.texname = '$\overline{q}_{max}$';
metr_qinfmax.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_qinfmax}];


% ***********************
%
% L_min, L_max
%

metr_Lmin.varname = 'Lmin';
metr_Lmin.texname = '$L_{min}$';
% metr_Lmin.format = format_default;
metr_Lmin.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_Lmin}];

metr_Lmax.varname = 'Lmax';
metr_Lmax.texname = '$L_{max}$';
% metr_Lmax.format = format_default;
metr_Lmax.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_Lmax}];

% ***********************
%
% D_min, D_max
%

metr_Dmin.varname = 'Dmin';
metr_Dmin.texname = '$D_{min}$';
% metr_Dmin.format = format_default;
metr_Dmin.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_Dmin}];

metr_Dmax.varname = 'Dmax';
metr_Dmax.texname = '$D_{max}$';
% metr_Dmax.format = format_default;
metr_Dmax.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_Dmax}];

% ***********************
%
% L/D_min, L/D_max
%

metr_LDmin.varname = 'LDmin';
metr_LDmin.texname = '$(L/D)_{min}$';
metr_LDmin.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_LDmin}];

metr_LDmax.varname = 'LDmax';
metr_LDmax.texname = '$(L/D)_{max}$';
metr_LDmax.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_LDmax}];


% ***********************
%
% loading_min, loading_max
%

metr_loadmin.varname = 'loadmin';
metr_loadmin.texname = '$(N/M)_{min}$';
metr_loadmin.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_loadmin}];

metr_loadmax.varname = 'loadmax';
metr_loadmax.texname = '$(N/M)_{max}$';
metr_loadmax.format = format_default;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_loadmax}];

% ***********************
%
% (\sum F_L)_min, (\sum F_L)_max
%

metr_sumFLmin.varname = 'sumFLmin';
metr_sumFLmin.texname = '$(\sum F_{L})_{min}$';
% metr_sumFLmin.format = format_default;
metr_sumFLmin.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_sumFLmin}];

metr_sumFLmax.varname = 'sumFLmax';
metr_sumFLmax.texname = '$(\sum F_{L})_{max}$';
% metr_sumFLmax.format = format_default;
metr_sumFLmax.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_sumFLmax}];


% ***********************
%
% (\sum F_D)_min, (\sum F_D)_max
%

metr_sumFDmin.varname = 'sumFDmin';
metr_sumFDmin.texname = '$(\sum F_{D})_{min}$';
% metr_sumFDmin.format = format_default;
metr_sumFDmin.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_sumFDmin}];

metr_sumFDmax.varname = 'sumFDmax';
metr_sumFDmax.texname = '$(\sum F_{D})_{max}$';
% metr_sumFDmax.format = format_default;
metr_sumFDmax.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_sumFDmax}];


% ***********************
%
% |G*(x)|_min, |G*(x)|_max
%
if do_ndi
metr_detGstarmin.varname = 'detGstarmin';
metr_detGstarmin.texname = '$|G^{*}(x)|_{min}$';
% metr_detGstarmin.format = format_default;
metr_detGstarmin.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_detGstarmin}];

metr_detGstarmax.varname = 'detGstarmax';
metr_detGstarmax.texname = '$|G^{*}(x)|_{max}$';
% metr_detGstarmax.format = format_default;
metr_detGstarmax.format = format_exp;

% Add to metric cell array
metric_cell = [metric_cell ; {metr_detGstarmax}];
end




%%
% *************************************************************************
% *************************************************************************
%
% READ PERFORMANCE METRIC DATA FROM FILE      
%
% *************************************************************************
% *************************************************************************

% tmp = load([relpath_data 'hsv_wang_stengel_2000_model.mat']);
% perf_metric_cell = tmp.perf_metric_cell;



%%
% *************************************************************************
% *************************************************************************
%
% TABULATE PERFORMANCE METRIC DATA     
%
% *************************************************************************
% *************************************************************************

% Number of metrics considered
nummetric = size(metric_cell,1);

% Holds numerical performance metric data
metricdatatmat = zeros(nummetric, numpresets);

% Holds formatted performance metric data (string)
metricdata_cell = cell(nummetric, numpresets);

% ***********************
%       
% MAIN LOOP
%

for presetcount = 1:numpresets

    % Get the performance metric struct for this preset
    currperfmetric_cell = perf_metric_cell{presetcount};

    for i = 1:nummetric
        
        % Get current metric object
        currmetric = metric_cell{i};

        % Check if the current preset has the current metric defined
        hascurrmetric = isfield(currperfmetric_cell, currmetric.varname);

        % Extract the numerical data for the current metric at the current
        % preset, if it is available
        if hascurrmetric
            currmetricdata = currperfmetric_cell.(currmetric.varname);
        else
            % Else, the current performance metric data was not available
            % for this preset. Mark the value as nan
            currmetricdata = nan;
        end

        % Format the current metric data to latex-syntax string
        if hascurrmetric
            if currmetricdata == inf
                currmetricdatastr = '$\infty$';
            elseif currmetricdata == -inf
                currmetricdatastr = '$-\infty$';
            else
                currmetricdatastr = ...
                    num2str(currmetricdata,currmetric.format);
            end
        else
            % Else, the current performance metric data was not available
            % for this preset. Write in 'N/A'
            currmetricdatastr = 'N/A';
        end

        % Write the data to the storage matrix
        metricdatatmat(i,presetcount) = currmetricdata;
            
        % Write the formatted data to the storage cell array
        metricdata_cell{i,presetcount} = currmetricdatastr;

    end

end