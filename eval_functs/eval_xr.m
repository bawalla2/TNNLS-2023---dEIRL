function xrt = eval_xr(t, xr_sett)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% EVALUATE REFERENCE COMMAND TRAJECTORY x_r(t)
%
% Brent Wallace  
%
% 2022-08-23
%
% This program, given a tag of a desired noise reference command and time t
% evaluates the probing command x_r(t).
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% xrt = eval_xr(t, xr_sett)
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% t         (Double) Time (sec)
% xr_sett        (Struct) Contains properties of the pecific signal to be
%           evaluated. Has the following fields:
%   tag     (String) Tag of the signal to be evaluated
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% xrt        (Vector) Evaluation of probing noise x_r(t)
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% BEGIN MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

switch xr_sett.tag

    % ***********************
    %
    % ZERO NOISE SIGNAL
    %
    
    case '0'
        
        xrt = 0;
 

    % ***********************
    %
    % NDI HSV -- WANG, STENGEL (2000) -- TRACKING
    %   
    
    case 'ndi_wang_stengel_2000_tracking'
        
        % Extract reference trajectory settings
        V_m = xr_sett.V_m;
        A_V = xr_sett.A_V;
        T_V = xr_sett.T_V;
%         omega_V = xr_sett.omega_V;
        g_m = xr_sett.h_m;
        A_g = xr_sett.A_h;
        T_h = xr_sett.T_h;
%         omega_h = xr_sett.omega_h;

        omega_V = 2 * pi / T_V;
        omega_g = 2 * pi / T_h;

        % Evaluate reference trajectory -- V_r(t)
        Vrt = V_m + A_V * sin(omega_V * t);
        dVrt = omega_V * A_V * cos(omega_V * t);
        ddVrt = -omega_V^2 * A_V * sin(omega_V * t);
        dddVrt = -omega_V^3 * A_V * cos(omega_V * t);

        % Evaluate reference trajectory -- h_r(t)
        grt = g_m + A_g * sin(omega_g * t);
        dgrt = omega_g * A_g * cos(omega_g * t);
        ddgrt = -omega_g^2 * A_g * sin(omega_g * t);
        dddgrt = -omega_g^3 * A_g * cos(omega_g * t);
        ddddgrt = omega_g^4 * A_g * sin(omega_g * t);

        % Pack output
        xrt = [ Vrt
                dVrt
                ddVrt
                dddVrt
                grt
                dgrt
                ddgrt
                dddgrt
                ddddgrt     
                ];


    % ***********************
    %
    % NDI HSV -- WANG, STENGEL (2000) -- TRACKING
    %   
    
    case 'ndi_wang_stengel_2000_sin_Vg'
        
        % Extract reference trajectory settings
        V_m = xr_sett.V_m;
        A_V = xr_sett.A_V;
        T_V = xr_sett.T_V;
%         omega_V = xr_sett.omega_V;
        g_m = xr_sett.g_m;
        A_g = xr_sett.A_g;
        T_g = xr_sett.T_g;
%         omega_h = xr_sett.omega_h;

        omega_V = 2 * pi / T_V;
        omega_g = 2 * pi / T_g;

        % Evaluate reference trajectory -- V_r(t)
        Vrt = V_m + A_V * sin(omega_V * t);
        dVrt = omega_V * A_V * cos(omega_V * t);
        ddVrt = -omega_V^2 * A_V * sin(omega_V * t);
        dddVrt = -omega_V^3 * A_V * cos(omega_V * t);

        % Evaluate reference trajectory -- h_r(t)
        grt = g_m + A_g * sin(omega_g * t);
        dgrt = omega_g * A_g * cos(omega_g * t);
        ddgrt = -omega_g^2 * A_g * sin(omega_g * t);
        dddgrt = -omega_g^3 * A_g * cos(omega_g * t);
        ddddgrt = omega_g^4 * A_g * sin(omega_g * t);

        % Pack output
        xrt = [ Vrt
                dVrt
                ddVrt
                dddVrt
                grt
                dgrt
                ddgrt
                dddgrt
                ddddgrt     ];        

    % ***********************
    %
    % NDI HSV -- WANG, STENGEL (2000) -- TRACKING
    %   
    
    case 'ndi_wang_stengel_2000_sum_sin_Vg'
        

        % Get settings
        V_m = xr_sett.V_m;
        g_m = xr_sett.g_m;
        cos1_sin0 = xr_sett.cos1_sin0;
        Amat = xr_sett.Amat;
        Tmat = xr_sett.Tmat;       
        dorefvec = xr_sett.dorefvec;
        nderivvec = xr_sett.nderivvec;
        m = size(Amat,1);
        nsin = size(Amat,2);
        nsig = sum(nderivvec + 1);

        % Initialize
        xrt = zeros(nsig,1);
        cnt = 1;
        
        for i = 1:m

            % Get count at beginning of this output
            cnt_0 = cnt;

            % Add constant term
            switch i
                case 1
                    xrt(cnt) = V_m;
                case 2
                    xrt(cnt) = g_m;
            end
            
            % Add sinusoidal terms in each derivative (if this output is
            % active)
            for j = 1:nsin

                % Reset counter to beginning
                cnt = cnt_0;

                % Pull settings for this output
                wij = 2*pi/Tmat(i,j);
                c1s0ij = cos1_sin0(i,j);
                c1s0ijk = c1s0ij;
                csgn = 1;

                % Counter to keep track of when to flip sign
                if c1s0ij
                    scnt = 2;
                else
                    scnt = 1;
                end 

                for k = 0:nderivvec(i)
                          
                    wijk = wij^k;


                    if dorefvec(i)
                        if c1s0ijk
                            tmp = cos(wij * t);
                        else
                            tmp = sin(wij * t);
                        end

                        tmp = Amat(i,j) * csgn * wijk * tmp; 
        
                        xrt(cnt) = xrt(cnt) +  tmp;
                    end

                    if mod(scnt,2) == 0
                        scnt = 0;
                        csgn = -csgn;
                    end

                    c1s0ijk = ~ c1s0ijk;

                    scnt = scnt + 1;
                    cnt = cnt + 1;

                end

               
            end

        end

        xrt = [ xrt(1)
                zeros(3,1)
                xrt(2)  ];

%         xrt(end) = xrt(end) ...
%                         + 0.5 * exp(-0.01*t) * sin(2*pi/10*t) * pi/180;
        
        
        


    % ***********************
    %
    % MULTI-STEP REFERENCE COMMAND
    %   
    
    case 'multistep'

        stptoffsetvec = xr_sett.stptoffsetvec;
        stpt_cell = xr_sett.stpt_cell;
        stptt_cell = xr_sett.stptt_cell;

        m = size(stptoffsetvec,1);

        yvec = zeros(m,1);

        for i = 1:m

            % Get current setpoint vector, setpoint time vector
            stptvec = stpt_cell{i};
            stpttvec = stptt_cell{i};

            % Number of setpoints for this output
            numstpt = size(stptvec,1);

            % Check time vector to see which setpoint to apply
            j = 1;
            loop = 1;
            while loop
                if j > numstpt-1
                    % Past last element of the array. Apply final setpoint
                    % value and break
                    yvec(i) = stptvec(j);
                    loop = 0;
                else
                    % Not yet at end of array. Check if this is the right
                    % time interval
                    tcheck = stpttvec(j);
                    if t < tcheck 
                        % Found proper interval. Set setpoint value and
                        % break
                        yvec(i) = stptvec(j);
                        loop = 0;
                    else
                        % Not yet foound the proper interval. Increment
                        % interval endpoint and keep checking     
                        j = j + 1;
                    end
                end              
            end

            % Add offset
            yvec(i) = yvec(i) + stptoffsetvec(i);

        end

        
        % Pack output
        xrt = [ yvec(1)
                0
                0
                0
                yvec(2)
                0
                0
                0
                0     ];    


 

    % *********************************************************************
    % *********************************************************************
    %
    % THROW ERROR IF TAG DOES NOT COME UP A MATCH
    %   
    % *********************************************************************
    % *********************************************************************
    
    otherwise
        
        error('*** ERROR: REFERENCE TRAJECTORY TAG NOT RECOGNIZED ***');

end