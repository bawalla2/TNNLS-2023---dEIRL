function figcount = plot_irl_hsv(alg_settings, out_data, group_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% PLOTS FOR INTEGRAL REINFORCEMENT LEARNING (IRL) ALGORITHM
%
% Brent Wallace  
%
% 2022-01-13
%
% This program handles plots specific to the data from the IRL algorithm
% presented in,
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
% figcount = plot_irl(alg_settings, out_data, group_settings)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% alg_settings      (Struct) Algorithm settings/parameters corresponding to
%                   the current preset. See respective algorithm .m-file
%                   for a description of the relevant fields.
%                   NOTE: Regardless of the algorithm, alg_settings must
%                   contain the following fields:
%   group_settings   (Struct) contains plot settings for this particular
%                   preset. Has the following fields:
%       relpath     (String) Name of the folder to save plots to for this
%                   preset. This could be the preset tag, or any
%                   other convenient identifier.
% out_data          (Struct) Output data generated by the algorithm. See
%                   respective algorithm .m-file for a description of the
%                   relevant fields.
% group_settings     (Struct) contains plot settings for the program. Has
%                   the following fields which are used here:
%   savefigs        (Boolean) 1 = save figures to PDF. 0 = don't save.
%   figcount        (Integer) Cumulative figure count before this function
%                   has been called.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% figcount          (Integer) Cumulative figure count after all plots in
%                   this function have been created.
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

% Unpack plot settings
savefigs = group_settings.savefigs;
if savefigs
    relpath = group_settings.relpath;
    relpath_alg = [relpath 'DIRL\'];
    mkdir(relpath_alg);
    relpath_data = [relpath_alg 'data\'];
    mkdir(relpath_data);
end
% dolegend = group_settings.dolegend;

% % Number of designs to plot for
% numpresets = size(alg_settings,1);

% Extract system and system plot settings
sys = alg_settings.sys;
sys_plot_settings = group_settings.sys_plot_settings;

% Initialize figure counter
figcount = group_settings.figcount;

% x-axis label for time plots
tlabel = group_settings.tlabel;

% State trajectory x(t) unit scaling. E.g., if x_1(t) is angular
% displacement but is desired in degrees, declare the field 'sclvec' and
% put sclvec(1) = 180/pi.
x_sclvec = sys_plot_settings.x_sclvec;


% Legend for all states in one plot
x_t_state_lgd = sys_plot_settings.x_t_state_lgd;

% Extract plot time window
tsim_plot = group_settings.tsim_plot;

% Save data filename
filename_data = group_settings.filename_data_dirl;

% % DIRL data
% dirl_data = out_data.dirl_data;


% *************************************************************************
% *************************************************************************
%
% UNPACK ALGORITHM OUTPUT DATA
%
% See respective algorithm .m-file for details of 'out_data' struct fields.
%
% *************************************************************************
% *************************************************************************

% tvec = out_data.tvec;
% xmat = out_data.xmat;
% umat = out_data.umat;

cmat_cell = out_data.cmat_cell;
P_cell = out_data.P_cell;
K_cell = out_data.K_cell;

% Conditioning data
cond_A_vec_cell = out_data.cond_A_vec_cell;

% Loop data
loopdata = out_data.loopdata;
numloops = loopdata.numloops;
doloopvec = loopdata.doloopvec;
istarvec = loopdata.istarvec;

% Maximum i^{*} among the active loops
maxistar =  loopdata.maxistar;

% Number of loops executed
numdoloops = sum(doloopvec);

% Learning trajectory data
tlvec = out_data.tlvec; 
ulmat = out_data.ulmat;
xlmat = out_data.xlmat;

% LQ data
lq_data = out_data.lq_data;

% DIRL data
dirl_data = out_data.dirl_data;

% Figure count at beginning
figcount_0 = figcount;

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

% Legend for loop entries
lgd_loop = cell(numdoloops,1);

% Active loop counter
doloopcnt = 0;

for k = 1:numloops

if doloopvec(k)

% Increment counter
doloopcnt = doloopcnt + 1;


% Determine legend entry
switch numloops
    case 1
        currtexname = '';
    case 2
        switch k
        
            case 1
                currtexname = '$V$';
        
            case 2
                currtexname = '$\gamma$';
        
        end
end

% Add this loop to the master legend
lgd_loop{doloopcnt} = currtexname;

% Extract current loop weight data
c_mat = cmat_cell{k};

% Extract current loop conditioning data
cond_A_vec = cond_A_vec_cell{k};

% Number of activation functions
Nk = size(c_mat,2);

% *************************************************************************
% *************************************************************************
%
% PLOT: CONDITION NUMBER VERSUS ITERATION COUNT
%
% *************************************************************************
% *************************************************************************

% PLOT
figure(figcount_0)
h_fig = semilogy(0:istarvec(k)-1, cond_A_vec(1:end)); 
set(h_fig, 'Marker', 'o');
hold on

% Do formatting if last loop
if doloopcnt == numdoloops

    ttl = ['Condition Number vs. Iteration Count'];  
    title(ttl)
    xlabel('Iteration $i$');
    ylabel('$\kappa(\Theta_{i})$');
    lgd = legend(lgd_loop);
    
    % x ticks
    xticks(0:maxistar-1)  
    xlim([0 maxistar-1]);
    
    % Format plot
    p_sett.figcount = figcount_0;
    plot_format(p_sett); 
    
    % SAVE PLOT
    if savefigs
        filename = ['cond_A_vs_i'];
        savepdf(figcount_0, relpath_alg, filename); 
    end

end

% Increment figure counter if this is the first time around
if doloopcnt == 1
    figcount = figcount + 1;
end


% *************************************************************************
% *************************************************************************
%
% PLOT: CRITIC NN PARAMETERS
%
% *************************************************************************
% *************************************************************************

% String v(P_{i,k})
switch numloops
    case 1
        strvPiknd = ['v(P_{i})'];
    case 2
        strvPiknd = ['v(P_{i,' num2str(k) '})'];
end
strvPik = ['$' strvPiknd '$'];

% Prepare legend entries
lgd_c = cell(Nk,1);
for j = 1:Nk
    lgd_c{j} = ['$' strvPiknd '_{' num2str(j) '}$'];
end

% PLOT
figure(figcount)
h_fig = plot(0:istarvec(k)-1, c_mat); 
set(h_fig, 'Marker', 'o');
switch numloops
    case 1
        title(['Weights ' strvPik]); 
    case 2
        title(['Weights ' strvPik ' -- ' currtexname]); 
end
xlabel('Iteration $i$'); 
ylabel(strvPik);
% If there aren't too many weights, include legend
if Nk <= 10
    lgd = legend(lgd_c);
end
% set(lgd, 'Numcolumns', 2);          % Make legend 2 columns
% set(lgd, 'Location', 'Best');       % Put legend in empty spot

% x ticks
xticks(0:istarvec(k)-1)  
xlim([0 istarvec(k)-1]);

% Format plot
p_sett.figcount = figcount;
plot_format(p_sett); 

% SAVE PLOT
if savefigs
    filename = ['vPi' num2str(k)];
    savepdf(figcount, relpath_alg, filename); 
end

% Increment figure counter
figcount = figcount + 1;

end

end



%%
% *************************************************************************
% *************************************************************************
%
% SAVE DIRL LEARNING DATA
%
% *************************************************************************
% *************************************************************************


% Save data
if savefigs

    % Save data 
    varname = 'dirl_data';
    save([relpath_data filename_data], varname);

end
    


