function figcount = plot_cond(alg_settings_cell_master,...
    out_data_cell_master, group_settings_master, master_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONDITIONING PLOTS
%
% Brent Wallace  
%
% 2022-01-13
%
% This program takes algorithm conditioning data for all of the algorithms
% tested (if applicable) and plots it all together in one plot.
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
savefigs = master_settings.savefigs;
if savefigs
    relpath = master_settings.relpath;
    relpath_alg = [relpath 'cond\'];
    mkdir(relpath_alg);
end


% Initialize figure counter
figcount = master_settings.figcount;

% Number of preset groups executed
numgroups = master_settings.numgroups;


% *************************************************************************
% *************************************************************************
%
% UNPACK ALGORITHM OUTPUT DATA
%
% See respective algorithm .m-file for details of 'out_data' struct fields.
%
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

% Plot formatting -- dashed line
psett_dash = {'LineStyle', '--'};

% Individual settings for each loop
indiv_sett_cell_loop = cell(2,1);
indiv_sett_cell_loop{1} = psett_dash;

% Master individual settings cell array
indiv_sett_cell_master = [];

% Master legend
lgd_master = [];

% Max i^* across all presets
maxistar = 0;

% Flag to keep track if old IRL data was plotted
hasoldirl = 0;

% Loop over the number of preset groups executed
for i = 1:numgroups

% Pull current preset group
groupseti = group_settings_master{i};

% Get output data
outdatai = out_data_cell_master{i};

% Get algorithm settings cell
algsettsi = alg_settings_cell_master{i};

% Get algorithm list
alg_list = groupseti.alg_list;

% Get plot color settings
color_sett_cell = groupseti.color_sett_cell;   
color_setti = color_sett_cell{1};

% Check if this group is a training group
istraining = groupseti.istraining;

% If is a training group, plot conditioning data
if istraining

    % ***********************
    %
    % UNPACK ALGORITHM OUTPUT DATA
    %

    % Get output data
    out_data = outdatai{1};

    % Get algorithm settings
    alg_settings = algsettsi{1};

    % Conditioning data
    cond_A_vec_cell = out_data.cond_A_vec_cell;
    
    % Loop data
    loopdata = out_data.loopdata;
    numloops = loopdata.numloops;
    doloopvec = loopdata.doloopvec;
    istarvec = loopdata.istarvec;
    
    % Number of loops executed
    numdoloops = sum(doloopvec);

    % Optimal LQ data -- simulation model
    lq_data_opt_sim = out_data.lq_data_opt_sim;

    % Controller data
    K_cell = out_data.K_cell;

    % Do reference command r(t) injection (=1) or not (=0)
    dort = alg_settings.dort;

    % Is DIRL (=1) or old IRL (=0)
    dirl1irl0 = groupseti.dirl1irl0;
    if ~dirl1irl0
        hasoldirl = 1;
    end

    % Legend entry for this group
    lgdi = alg_list{1};

    % ***********************
    %
    % PLOT
    %

    for k = 1:numloops

        % If loop executed, plot data
        if doloopvec(k)

        % Get conditioning in this loop
        cond_A_vec = cond_A_vec_cell{k};

        % PLOT
        figure(figcount)
        h_fig = semilogy(0:istarvec(k)-1, cond_A_vec(1:end)); 
        set(h_fig, 'Marker', 'o');
        hold on

        % Legend entry for this loop
        lgdik = lgdi;

%         % Check if is DIRL without r(t) injection
%         if dirl1irl0 && ~dort
%             rtstrng = ' (no $r(t)$)';
%         else
%             rtstrng = '';
%         end
%         lgdik = [lgdik rtstrng];

        % Insert appropriate loop indicator in legend entry
        switch numloops
            case 1
                
            case 2
                switch k
                
                    case 1
                        currtexname = '$V$';
                
                    case 2
                        currtexname = '$\gamma$';
                
                end

                lgdik = [lgdik ' -- ' currtexname];
            
        end

        % Append legend entry to master list
        lgd_master = [lgd_master; {lgdik}];

        % Update maximum i^*
        maxistar = max([maxistar istarvec(k)]);

        % Individual settings for this loop
        indiv_settik = color_setti;
        if numloops > 1
            indiv_settik = [indiv_settik; indiv_sett_cell_loop{k}];
        end

        % Append individual settings to master list
        indiv_sett_cell_master = [indiv_sett_cell_master; {indiv_settik}];

        % DISPLAY MAX/MIN CONDITIONING DATA
        [maxcond, indmaxcond] = max(cond_A_vec);
        [mincond, indmincond] = min(cond_A_vec);
        disp('*****')
        disp(['PRESET:   ' lgdik])
        disp(['MAX cond(A_i) =      ' num2str(maxcond-1) ...
            '    AT i =  ' num2str(indmaxcond-1)])
        disp(['MIN cond(A_i) =      ' num2str(mincond-1) ...
            '    AT i =  ' num2str(indmincond-1)])   
      

        % DEBUGGING: Display final controller error
        Kistark = K_cell{k}(:,:,end);
        switch numloops
            case 1
                Kkstar = lq_data_opt_sim.Kcirl;
            case 2
                switch k
                    case 1
                        Kkstar = lq_data_opt_sim.lq_data_11.K;
                    case 2
                        Kkstar = lq_data_opt_sim.lq_data_22.K;
                end
        end
        eKk = Kistark - Kkstar;
        normeKk = norm(eKk);
        disp(['||K_{i*,j} - K_j^*|| =   ' num2str(normeKk)])

        end

    end

    
    


end


end


% ***********************
%
% FORMATTING
%
% If anything was plotted, do formatting
%

if maxistar > 0

    ttl = ['Condition Number vs. Iteration Count'];  
    title(ttl)
    xlabel('Iteration $i$');
    ylabel('$\kappa(\Theta_{i,j})$');
    lgd = legend(lgd_master);
    
    % x ticks
    xticks(0:maxistar-1)  
    xlim([0 maxistar-1]);

    % Manually set the legend position if this has old IRL data ('best'
    % still covers up old IRL data)
    if hasoldirl
        p_sett.custom_sett.lgd_position = [0.6245 0.4661 0.2645 0.3367];
    end
    
    % Format plot
    p_sett.figcount = figcount;
    p_sett.indiv_sett_cell = indiv_sett_cell_master;
    plot_format(p_sett); 
    
    % SAVE PLOT
    if savefigs
        filename = ['cond_A_vs_i'];
        savepdf(figcount, relpath_alg, filename); 
    end
    
    % Increment figure counter
    figcount = figcount + 1;

end
