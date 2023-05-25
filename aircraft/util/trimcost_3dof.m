function J = trimcost_3dof(optparam)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% COST FUNCTION FOR LONGITUDINAL TRIM CALCULATIONS
%
% Brent Wallace,
%
% Code inspired by Robert Stengel
%
% 2022-03-25
%
% This function, given a vector of vehicle states, evaluates the vehicle
% dynamical equations and assigns a penalty to the associated deviation
% from trim. 
%
% To be used in conjunction with MATLAB's fminsearch, cf.
%
% https://www.mathworks.com/help/matlab/ref/fminsearch.html#bvadxhn-1-options
%
% *************************************************************************
%
% CALL SYNTAX
%
% *************************************************************************
%
% function J = trimcost_3dof(optparam)
%
% NOTE: Before running this program, make sure the MATLAB directory is at
% the folder housing 'main.m'
%
% *************************************************************************
%
% INPUTS
%
% *************************************************************************
%
% optparam          (3-dim Vector) Vector of optimization parameters:
%                       dT      Throttle setting
%                       dE      Elevator deflection (rad)
%                       alpha   AOA (rad)
%
% *** NOTE:
%
% This function requires access to the global variable 'sys', which is the
% system structure. The struct 'sys' must have a subfield 'sys.model' with
% the following fields:
%
% fx                (Inline Function) f: R^n -> R^n system dynamical
%                   function. Note that the state vector must be laid out
%                   as
%                   x = [V, \gamma, h, \alpha, q]^T in R^{5} 
% gx                (Inline Function) g: R^n -> R^{nxm} system input gain
%                   matrix function
% trimconds         (Struct) Has the following fields:
%   V0              (Double) Desired trim airspeed
%   gamma0          (Double) Desired trim FPA
%   h0              (Double) Desired trim altitude
%
%
% *************************************************************************
%
% OUTPUTS
%
% *************************************************************************
%
% J                 (double) Positive penalty on the deviation from trim
%                   conditions. Is calculated as:
%                       J = xcost^T * QR * xcost
%                   where xcost = [\dot{V}, \dot{\gamma}, \dot{q}] \in R^3,
%                   and Q = Q^T > 0 (in this case, we choose Q = I).
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
%
% GLOBAL VARIABLES
% 
% *************************************************************************

global sys;


% *************************************************************************
%
% CALCULATE STATE DERIVATIVE
% 
% *************************************************************************

% Q = Q^T > 0
Q = eye(3);

% Control candidate
u = optparam(1:2);

% State candidate
% x = [V, \gamma, h, \alpha, q]^T in R^{5} 
V0 = sys.model.trimconds.V0;
gamma0 = sys.model.trimconds.gamma0;
h0 = sys.model.trimconds.h0;

x = [   V0
        gamma0
        h0
        optparam(3);
        0               ];

				
% Evaluate system dynamics
f_x = sys.model.fx(x);
g_x = sys.model.gx(x);
xdot = f_x + g_x * u;

% Penalized state derivatives
% xcost = [\dot{V}, \dot{\gamma}, \dot{q}] \in R^3
xcost = [   xdot(1)
            xdot(2)
            xdot(5)     ];

% Cost function
J = xcost' * Q * xcost;
