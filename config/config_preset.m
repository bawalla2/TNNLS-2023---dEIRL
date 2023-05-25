function alg_settings = config_preset(preset, group_settings)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% SELECT ALGORITHM, SYSTEM, DESIGN PARAMETERS BASED ON PRESET
%
% Brent Wallace  
%
% 2021-11-06
%
% This program, given a desired preset, handles all algorithm
% initialization/configuration. This is the main program for configuring
% specific algorithm hyperparameters (e.g., sample rates, excitations,
% etc.)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% preset            (String) Algorithm/system preset for desired example.
% group_settings    (Struct) Contains system/design parameters to
%                   be shared across all presets in the desired group.
%                   E.g., if for this preset group all designs share the
%                   same Q, R matrices, those fields may be included in
%                   this struct. This is initialized in
%                   config_preset_group.m.
% master_settings   (Struct) Master settings as initialized by main.m.
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% alg_settings  (Struct) Algorithm settings/parameters for subsequent
%               execution according to desired preset (see respective
%               algorithm .m-file for details). 
%               
%
% NOTE: These are initialized automatically by settings in
% config_preset_group, but they nevertheless may be manually overriden.
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
sys = group_settings.sys;

% Degree/radian conversions
D2R =   pi/180;
R2D =   180/pi;


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% CONFIGURE PRESET
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

switch preset

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
    % HSV -- WANG, STENGEL (2000) -- IRL INNER/OUTER
    %
    
    case [{'dirl_lin'};{'dirl_nonlin'};{'irl_old'}]

        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = preset;
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       
        
%         % Controller/model initialization settings
%         relpath_ctrl = group_settings.relpath_ctrl;

        % Get preset count
        presetcount = group_settings.presetcount;

        % Commanded airspeed, altitude
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);

        % Plant model MP (=1) or NMP (=0)
        Pmp1nmp0 = group_settings.Pmp1nmp0vec(presetcount);

        % Algorithm is NDI (=1) or not NDI (=0)
        ndi1notndi0 = group_settings.ndi1notndi0vec(presetcount);

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);

        % Prefilter pole locations
        if pf1nopf0
            pfavec = group_settings.pfavec;   
        end

        % Is training (=1) or not (=0)
        istraining = group_settings.istraining;
   
        % Is DIRL (=1) or old IRL (=0)
        dirl1irl0 = group_settings.dirl1irl0;

        % Initial condition
        x0 = group_settings.x0;        
       
        % Time to simululate for 
        tsim = group_settings.tsim;

        % IRL settings
        irl_setts = group_settings.irl_setts_cell{presetcount};

        % Number of loops executed for this preset
        numloops = irl_setts.numloops;

        % Which loops to train
        doloopvec = irl_setts.doloopvec;

        % ***********************
        %
        % GET CONTROLLER SETTINGS
        %   

        lq_data = group_settings.lq_data;
        lq_data_nu = group_settings.lq_data_nu;

        % ***********************
        %
        % LOOP SETTINGS
        %   

        % Holds loop settings
        loop_cell = cell(numloops,1);
        
        % Initialize loop settings
        switch numloops

            % ***********************
            %
            % SINGLE-LOOP
            %   

            case 1

            % Sample period
            if dirl1irl0
                Ts = 5;
            else
                Ts = 0.15;
            end


            tmp.indsx = (1:4)';
            tmp.indsy = (1:2)';
            tmp.Q = lq_data.Qirl;
            tmp.R = lq_data.R;
            tmp.K0 = lq_data.Kdirl;
            
            tmp.istar = 5;
            tmp.nTs = 1;    
            tmp.l = 25;             

            % Set current loop settings
            loop_cell{1} = tmp;

            % ***********************
            %
            % DOUBLE-LOOP
            %   

            case 2

                % Sample period
                Ts = 2;

            for k = 1:numloops
                switch k
    
                    % V loop
                    case 1
                        tmp.indsx = 1;
                        tmp.indsy = 1;
                        tmp.Q = lq_data.lq_data_V.Q;
                        tmp.R = lq_data.lq_data_V.R;
                        tmp.K0 = lq_data.lq_data_V.K;
                        
                        tmp.istar = 5;
                        tmp.nTs = 6 / Ts;    
                        tmp.l = 15;
    
                    % \gamma loop    
                    case 2
                        tmp.indsx = (2:4)';
                        tmp.indsy = 2;
                        tmp.Q = lq_data.lq_data_g.Q;
                        tmp.R = lq_data.lq_data_g.R;
                        tmp.K0 = lq_data.lq_data_g.K;
    
                        tmp.istar = 5;
                        tmp.nTs = 1;
                        tmp.l = 25;
    
    
                end
    
                % Set current loop settings
                loop_cell{k} = tmp;
            end

        end

        % ***********************
        %
        % EXPLORATION NOISE
        %
        % Note: In post-transformation units (i.e., after applying S_u)
        %

        % ORIGINAL
        noise.cos1_sin0 = [ 1   0   0   0
                            1   0   0   1    ];
        noise.Amat = [      0.2 0.1 0.1 0
                            0   10   2.5  5  ];
        noise.Tmat = [      Inf 250 25  50
                            Inf 6  25   50  ];
        switch numloops
            case 1
                noise.donoisevec = ones(2,1);
            case 2
                noise.donoisevec = doloopvec;
        end
        noise.tag = 'multivar_sum_sinusoid';        
%         noise.tag = '0';

        % If is old IRL, don't insert noise
        if ~dirl1irl0
            noise.tag = '0';
        end


        % ***********************
        %
        % AIRSPEED/FPA REFERENCE COMMANDS -- FOR TRAINING PHASE
        %


        % Determine whether or not to insert r(t)
        hasdort = isfield(group_settings, 'dort');
        if hasdort
            dort = group_settings.dort;
        else
            dort = 1;
        end

        % Proceed to insert r(t) if desired 
        if dort
            refcmdl = 'sum_sin';
        else
            refcmdl = 'step';
        end


        % Trim
        x_e = sys.model.trimconds.xe_dE;

        % Trim airspeed (ft/s)
        V_e = x_e(sys.model.indV);

        % Trim FPA (rad)
        g_e = x_e(sys.model.indg);

        switch refcmdl

            % NO r(t)
            case 'step'
        
                Vr = 0;
                gr = 0;                
        
                % Set params
                V_m = V_e + Vr;
                A_V = 0;
                T_V = 1;
                g_m = g_e + gr;
                A_g = 0;
                T_g = 1;          

            % INSERT r(t)
            case 'sum_sin'
                
                V_m = V_e;
                g_m = g_e;       
%                 nderivvec = [3; 4];
                nderivvec = [0; 0];
 
                % ORIGINAL
                cos1_sin0 = [   0   0   1  
                                0   0   1   ];
                Amat = [        50  10   10   
                                2.5 1 0.2   ];
                Tmat = [        200 25  10  
                                15  6   3     ];


                Amat(2,:) = 0.1 * Amat(2,:);

                % Convert \gamma amplitudes to rad
                Amat(2,:) = D2R * Amat(2,:);


        end


        % Do x_3 loop
        switch numloops
            case 1
                dox3 = 1;  
            case 2
                dox3 = 1;  
        end

        % If executing old IRL algorithm, perturb the IC
        if ~dirl1irl0
%         if 1
            x0(sys.model.indV) = x0(sys.model.indV) + 1000;
            x0(sys.model.indg) = x0(sys.model.indg) + D2R * 2;
        end
                  


        % ***********************
        %
        % TRANINING SETTINGS
        %

        % Overwrite current controller (=1) or not (=0)
        updatecontroller = istraining;


        % Use the nominal linearization A for w = f(x) - A x at nominal
        % trim (=1) or simulation trim (=0)
        wnom1sim0 = 1;


        % Commanded airspeed, FPA
        switch refcmdl
            case 'step'
                r_sett_train.tag = 'ndi_wang_stengel_2000_sin_Vg';
                r_sett_train.Vr = Vr;
                r_sett_train.gr = gr;                
            case 'multistep'
                r_sett_train.tag = 'multistep';
            case 'sin'
                r_sett_train.tag = 'ndi_wang_stengel_2000_sin_Vg';   
            case 'sum_sin'
                r_sett_train.tag = 'ndi_wang_stengel_2000_sum_sin_Vg'; 
        end
        switch refcmdl
            case 'multistep'
                r_sett_train.stptoffsetvec = stptoffsetvec;
                r_sett_train.stpt_cell = stpt_cell;
                r_sett_train.stptt_cell = stptt_cell;
            case 'sum_sin'
                r_sett_train.V_m = V_m;
                r_sett_train.g_m = g_m;
                r_sett_train.nderivvec = nderivvec;
                r_sett_train.dorefvec = noise.donoisevec;
                r_sett_train.cos1_sin0 = cos1_sin0;
                r_sett_train.Amat = Amat;
                r_sett_train.Tmat = Tmat;
            otherwise                
                r_sett_train.V_m = V_m;
                r_sett_train.A_V = A_V;
                r_sett_train.T_V = T_V;
                r_sett_train.g_m = g_m;
                r_sett_train.A_g = A_g;
                r_sett_train.T_g = T_g;
                r_sett_train.tag = 'ndi_wang_stengel_2000_sin_Vg';
        end
        



        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;           

    % *********************************************************************
    %
    % HSV -- WANG, STENGEL (2000) LQ SERVO INNER/OUTER
    %
    
    case 'wang_stengel_2000_lq_servo_inout'
    
        % ***********************
        %
        % ALGORITHM
        %        
        
        alg = 'wang_stengel_2000_lq_servo_inout';
  
        % ***********************
        %
        % SETTINGS AND DESIGN PARAMETERS
        %       
        
%         % Controller/model initialization settings
%         relpath_ctrl = group_settings.relpath_ctrl;

        % Get preset count
        presetcount = group_settings.presetcount;

        % Commanded airspeed, altitude
        r_sett = group_settings.r_sett;

        % Model linear (=1) or nonlinear (=0)
        lin1nonlin0 = group_settings.lin1nonlin0vec(presetcount);

        % If model linear, use coupled model (=1) or decoupled (=0)
        c1dc0 = group_settings.c1dc0vec(presetcount);

        % Plant model MP (=1) or NMP (=0)
        Pmp1nmp0 = group_settings.Pmp1nmp0vec(presetcount);

        % Algorithm is NDI (=1) or not NDI (=0)
        ndi1notndi0 = group_settings.ndi1notndi0vec(presetcount);

        % Do prefilter (=1) or not (=0)
        pf1nopf0 = group_settings.pf1nopf0vec(presetcount);

        % Prefilter pole locations
        if pf1nopf0
            pfavec = group_settings.pfavec;   
        end
   

        % Initial condition
        x0 = group_settings.x0;        
       
        % Time to simululate for 
        tsim = group_settings.tsim;

        % Get algorithm names
        algnames = group_settings.algnames;
        
        % Get list of algorithms executed
        alg_list = group_settings.alg_list;

        % Get name of current algorithm
        curralg = alg_list{presetcount};

        % Nominal, simulation model
        switch curralg
            case algnames.lq_opt_nu
                alg_settings.model_nom_tag = group_settings.model_sim_tag;
                alg_settings.model_sim_tag = group_settings.model_sim_tag;
            otherwise
                alg_settings.model_nom_tag = group_settings.model_nom_tag;
                alg_settings.model_sim_tag = group_settings.model_sim_tag;                
        end


        % ***********************
        %
        % GET CONTROLLER SETTINGS
        %   

%         lq_data = group_settings.lq_data;
        
        switch curralg
            case algnames.lq
                K = group_settings.lq_data.K;
            case algnames.lq_opt_nu
                K = group_settings.lq_data_nu.Kjj;
        end


        % ***********************
        %
        % PLOT SETTINGS
        %    
        
        % Legend entry
        legend_entry = group_settings.lgd_p{presetcount};

        % Plot folder name. Can use preset tag, or anything else.
        plotfolder = legend_entry;           

 

    % *********************************************************************    
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    % *********************************************************************
    % *********************************************************************
    
    otherwise
        
        error('*** ERROR: PRESET TAG NOT RECOGNIZED ***');  
       
end





%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% STORE ALGORITHM SETTINGS/DESIGN PARAMETERS
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% GENERAL SETTINGS
%
% *************************************************************************

% Preset tag
alg_settings.preset = preset;

% Plot settings -- general
plot_settings.legend_entry = legend_entry;
plot_settings.plotfolder = plotfolder;
% plot_settings.sys_settings = sys_settings;

% Write plot settings
alg_settings.plot_settings = plot_settings;


% *************************************************************************
%
% ALGORITHM-SPECIFIC SETTINGS
%
% *************************************************************************

switch alg

    % *********************************************************************
    %
    % HSV -- WANG, STENGEL (2000) -- DIRL
    %
    
    case [{'dirl_lin'};{'dirl_nonlin'};{'irl_old'}]

        alg_settings.sys = sys;
        alg_settings.alg = alg;

        % Output variables to track
        alg_settings.inds_xr = group_settings.inds_xr;

        % Do learning (=1) or not (=0)
        alg_settings.dolearning = group_settings.dolearning;

        % Whether or not to perform learning in each loop
        alg_settings.doloopvec = irl_setts.doloopvec;

        % Do post-learning sim (=1) or not (=0)
        alg_settings.dopostlearning = group_settings.dopostlearning;         

        % Load data location, filename
        alg_settings.relpath_data = group_settings.relpath_data_dirl;
        alg_settings.filename_data = group_settings.filename_data_dirl;

        % Nominal model
        alg_settings.model_nom_tag = group_settings.model_nom_tag;

        % Simulation model
        alg_settings.model_sim_tag = group_settings.model_sim_tag;
        
        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = lin1nonlin0;

        % Plant model MP (=1) or NMP (=0)
        alg_settings.Pmp1nmp0 = Pmp1nmp0;

        % Algorithm is NDI (=1) or not NDI (=0)
        alg_settings.ndi1notndi0 = ndi1notndi0;

        % Do prefilter (=1) or not (=0)
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations
        if pf1nopf0
            alg_settings.pfavec = pfavec;   
        end

        % Exploration noise
        alg_settings.noise = noise;    

        % Commanded airspeed, FPA -- training phase
        alg_settings.r_sett_train = r_sett_train;

        % Commanded airspeed, FPA
        alg_settings.r_sett = r_sett;

        % Loop settings
        alg_settings.loop_cell = loop_cell;

        % Sample period
        alg_settings.Ts = Ts;

        % Do x_3 loop
        alg_settings.dox3 = dox3;

        % Is training preset (=1) or not (=0)
        istraining = group_settings.istraining;

        % Do reference command r(t) injection (=1) or not (=0)
        alg_settings.dort = dort;

        % Overwrite current controller (=1) or not (=0)
        alg_settings.updatecontroller = updatecontroller;

  
        % Use the nominal linearization A for w = f(x) - A x at nominal
        % trim (=1) or simulation trim (=0)
        alg_settings.wnom1sim0 = wnom1sim0;

        % Optimal LQ data
        alg_settings.lq_data_opt = lq_data;
        alg_settings.lq_data_opt_nu = lq_data_nu;

        % ICs, simulation time
        alg_settings.x0 = x0;  
        alg_settings.tsim = tsim; 

        % Figures/data path
        alg_settings.savefigs = group_settings.savefigs;
        alg_settings.relpath = group_settings.relpath;


    % *********************************************************************
    %
    % HSV -- WANG, STENGEL (2000) -- LQ SERVO INNER/OUTER
    %
    
    case 'wang_stengel_2000_lq_servo_inout'

        alg_settings.sys = sys;
        alg_settings.alg = alg;

        % Output variables to track
        alg_settings.inds_xr = group_settings.inds_xr;

        % Model linear (=1) or nonlinear (=0)
        alg_settings.lin1nonlin0 = lin1nonlin0;

        % If model linear, use coupled model (=1) or decoupled (=0)
        alg_settings.c1dc0 = c1dc0;

        % Plant model MP (=1) or NMP (=0)
        alg_settings.Pmp1nmp0 = Pmp1nmp0;

        % Algorithm is NDI (=1) or not NDI (=0)
        alg_settings.ndi1notndi0 = ndi1notndi0;

        % Do prefilter (=1) or not (=0)
        alg_settings.pf1nopf0 = pf1nopf0;

        % Prefilter pole locations
        if pf1nopf0
            alg_settings.pfavec = pfavec;   
        end

        % Commanded airspeed, FPA
        alg_settings.r_sett = r_sett;

        % Controller
        alg_settings.K = K;

        alg_settings.x0 = x0;  
        alg_settings.tsim = tsim; 



    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    
    otherwise
        
        error('**** ERROR: ALGORITHM TAG NOT RECOGNIZED ***');  
       
end
