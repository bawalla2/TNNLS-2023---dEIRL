function hsv_wang_stengel_2000_init(nu)
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE HSV PITCH-AXIS MODEL
%
% Brent Wallace  
%
% 2022-09-28
%
% This program initializes aerodynamic and other model data for a
% pitch-axis HSV model. Data is from:
%
%   Q. Wang and R. F. Stengel. "Robust nonlinear control of a hypersonic
%   aircraft." AIAA J. Guid., Contr., & Dyn., 23(4):577–585, July 2000
%
% And:
%
%   C. I. Marrison and R. F. Stengel. "Design of robust control systems for
%   a hypersonic aircraft." AIAA J. Guid., Contr., & Dyn., 21(1):58–63,
%   Jan. 1998.
%
% Which get their aerodynamic data from
%
% J. D. Shaughnessy, S. Z. Pinckney, J. D. McMinn, C. I. Cruz, and M.-L.
% Kelley. "Hypersonic vehicle simulation model: Winged-cone configuration."
% NASA TM-102610, Nov. 1990.
%
% NOTE: Make sure the MATLAB path is in the folder holding main.m before
% running this program.
%       
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


% % % ***********************
% % %
% % % CLEAR VARIABLES, COMMAND WINDOW, FIGURES
% % %
% % clear
% % clc
% % close all

% *************************************************************************
%
% GLOBAL VARIABLES
% 
% *************************************************************************

global sys;

% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************


% *************************************************************************
% *************************************************************************
%
% VEHICLE PERTURBATION VALUES
%
% Default model corresponds to \nu = 1
%
% *************************************************************************
% *************************************************************************

% Now declared as input args



% *************************************************************************
% *************************************************************************
%
% SETTINGS
%
% *************************************************************************
% *************************************************************************

% % Do symbolic confirmation of the linearization (=1) or don't do (=0)
% do_hand_confirmation = 1;

% Number of inputs
m = 2;

% *************************************************************************
% *************************************************************************
%
% INITIALIZE VEHICLE PARAMETERS
%
% *************************************************************************
% *************************************************************************

% Gravitational field constant (ft/s^2)
g = 32.1741;

% Gravitational constant (ft^3/s^2)
% Cf. Marrison, Nomenclature
mu = 1.39e16;

% Radius of earth (ft)
% Cf. Marrison, Nomenclature
RE = 20903500;

% Vehicle mass (slug)
% Cf. Marrison, Nomenclature
mref = 9375;

% Vehicle weight (lb)
wref = mref * g;

% Pitch-axis moment of inertia (slug-ft^2)
% Cf. Marrison, Nomenclature
Iyy = 7e6;

% Reference area (ft^2)
% Cf. Marrison, Nomenclature
S = 3603;

% MAC (ft)
% Cf. Marrison, Nomenclature
cbar = 80;

% Actuator model parameters

% % Cf. Wang Eqn. (35)
% k1 = 0;
% k2 = 0;
% k3 = 1;

zeta = 0.9;
omega_n = 10;

k1 = - 2 * zeta * omega_n;
k2 = - omega_n^2;
k3 = omega_n^2;        

% % % Testing out actuator response
% % % x = [\delta_{T}, \dot{\delta}_{T}]
% % % u = \delta_{T,com}
% % AT = [ 0, 1 ; -omega_n^2, - 2*zeta*omega_n];
% % BT = [0 ; omega_n^2];
% % CT = [1 0];
% % DT = 0;
% % H = ss(AT,BT,CT,DT);
% % x0 = [0.183; 0];
% % tvec = (0:0.01:5)';
% % uvec = 0.183*ones(length(tvec),1);
% % figure(100)
% % lsim(H,uvec,tvec,x0)

% % Store params
% sys.params.g = g;
% sys.params.mu = mu;
% sys.params.RE = RE;
% sys.params.mref = mref;
% sys.params.wref = wref;
% sys.params.Iyy = Iyy;
% sys.params.S = S;
% sys.params.cbar = cbar;
% sys.params.k1 = k1;
% sys.params.k2 = k2;
% sys.params.k3 = k3;



% *************************************************************************
% *************************************************************************
%
% INITIALIZE VEHICLE AERODYNAMIC COEFFICIENT FUNCTIONS
%
% *************************************************************************
% *************************************************************************


% ***********************
%       
% DEFINE SYMBOLIC VARIABLES
%      

% x = [V, \gamma, h, \alpha, q] in R^{5}
xs = sym('x', [1 5], 'real')';

% x_v = [V, \gamma, h, \alpha, q, \delta_{T}, \dot{\delta}_{T}] in R^{7}
xvs = sym('xv', [1 7], 'real')';

% u in R^{m}
us = sym('u', [1 2], 'real')'; 

% [x_v, u] in R^{7+m}
xvus = [    xvs
            us   ];

% Index variables (i.e., where each state is in x)
indV = 1;
indg = 2;
indh = 3;
inda = 4;
indq = 5;
inddT = 6;
indddT = 7;
% indVI = 8;
% indhI = 9;

% Index variables (i.e., where each control is in [xv u])
inddTc = 8;
inddE = 9;

% Ordered indices corresponding to
% z = [V, \gamma, \alpha, \delta_{T}, h] in R^{5}
% Cf. Wang, Eqn. (39)
indsz = [   indV
            indg
            inda
            inddT
            indh    ];

% Total vehicle state (i.e., with actuator dynamics), with zeros plugged
% into the actuator dynamics terms
xvs0(xs) = [    xs
                zeros(2,1); ];


% ***********************
%       
% MISC
%   

% Degree/radian conversions
D2R = pi/180;
R2D = 180/pi;

% sin(\alpha), cos(\alpha)
sa(xvs) = sin(xvs(inda));
ca(xvs) = cos(xvs(inda));

% sin(\gamma), cos(\gamma)
sg(xvs) = sin(xvs(indg));
cg(xvs) = cos(xvs(indg));

% sin(\alpha+\gamma), cos(\alpha+\gamma)
sapg(xvs) = sin(xvs(inda) + xvs(indg));
capg(xvs) = cos(xvs(inda) + xvs(indg));


% *************************************************************************
%
% AERO FUNCTIONS
% 
% *************************************************************************

% ***********************
%       
% TRIM CONDITIONS
%   

% Trim AOA (rad)
alpha_er = 0.0315;

% DEBUGGING: TEST TRIM
% Trim conditions
% x_v = [V, \gamma, h, \alpha, q, \delta_{T}, \dot{\delta}_{T}] in R^{7}
xve = [15060 ; 0 ; 110e3 ; alpha_er ; 0 ; 0.183 ; 0];
ue_ws = [0.183 ; -0.0066];

% ***********************
%       
% TOTAL DISTANCE FROM EARTH CENTER TO VEHICLE
%  

% Cf. Wang, Eqn. (30)
r(xvs) = RE + xvs(indh);
% DEBUGGING: Evaluate at trim
re = double(subs(r,xvs,xve))


% ***********************
%       
% AIR DENSITY \rho
%  

% Cf. Wang, Eqn. (A7)
rho(xvs) = 0.00238 * exp(-xvs(indh)/24000);
% DEBUGGING: Evaluate at trim
rhoe = double(subs(rho,xvs,xve))


% ***********************
%       
% SPEED OF SOUND a
%  

% Cf. Wang, Eqn. (A5)
a(xvs) = 8.99e-9*xvs(indh)^2 - 9.16e-4*xvs(indh) + 996;
% DEBUGGING: Evaluate at trim
ae = double(subs(a,xvs,xve))

% ***********************
%       
% MACH NUMBER M
%  

% Cf. Wang, Eqn. (A6)
M(xvs) = xvs(indV) / a;
% DEBUGGING: Evaluate at trim
Me = double(subs(M,xvs,xve))


% ***********************
%       
% LIFT COEFFICIENT C_L
%  

% Cf. Wang, Eqn. (A8)
CL(xvs) = xvs(inda) * (0.493 + 1.91 / M);
% DEBUGGING: Evaluate at trim
CLe = double(subs(CL,xvs,xve))

% Perturbation
CL_nu = nu.CL * CL;


% ***********************
%       
% LIFT COEFFICIENT C_L -- WITH NONMINIMUM PHASE BEHAVIOR ADDED
%
%   C_L = C_L(\alpha) + C_L(\delta_E)
%
% Where C_L(\alpha) is the main term defined above, and C_L(\delta_E) is
% the elevator-induced lift coefficient (not modeled by Wang and Stengel,
% added in here). New term fit over Shaughnessy data as
%
%   C_L(\delta_E) = (c1 \alpha^2 + c2 \alpha + c3) * \delta_E
%
% See fit_CL_de.m.
%  

% % Constants -- fit over \delta_E = {-10deg,10deg}
% CLdE_c1 = -0.449769705479619;
% CLdE_c2 = -0.0138761375623472;
% CLdE_c3 = -0.0161431704966951;

% Constants -- fit over \delta_E = all points
CLdE_c1 = -0.235580737050513;
CLdE_c2 = -0.00451812931308866;
CLdE_c3 = -0.0291335007132150;

% C_L(\delta_E)
CLdEu(xvs) = (CLdE_c1*xvs(inda)^2 + CLdE_c2*xvs(inda) + CLdE_c3);
CLdE(xvus) = CLdEu * xvus(inddE);

% DEBUGGING: Evaluate at trim
CLdEue = double(subs(CLdEu,xvs,xve))

% ***********************
%       
% DRAG COEFFICIENT C_D
%  

% Cf. Wang, Eqn. (A9)
CD(xvs) = 0.0082 * (171*xvs(inda)^2 + 1.15*xvs(inda) + 1) * ...
            (0.0012*M^2 - 0.054*M + 1);


% DEBUGGING: Evaluate at trim
CDe = double(subs(CD,xvs,xve))


% ***********************
%       
% PITCHING MOMENT COEFFICIENT -- MAIN TERM C_M(\alpha)
%  

% Cf. Wang, Eqn. (A11)
CMalpha(xvs) = 1e-4 * (0.06 - exp(-M/3)) ...
    * (-6565 * xvs(inda)^2 + 6875*xvs(inda) + 1);


% DEBUGGING: Evaluate at trim
CMalphae = double(subs(CMalpha,xvs,xve))


% ***********************
%       
% PITCHING MOMENT COEFFICIENT -- ROTATIONAL DAMPING C_M(q)
%  

% Cf. Wang, Eqn. (A12)
CMq(xvs) = cbar / (2*xvs(indV)) * xvs(indq) * (-0.025*M + 1.37) * ...
    (-6.83*xvs(inda)^2 + 0.303*xvs(inda) - 0.23);
% DEBUGGING: Evaluate at trim
CMqe = double(subs(CMq,xvs,xve))


% ***********************
%       
% PITCHING MOMENT COEFFICIENT -- ELEVATOR-INDUCED PITCHING COEFFICIENT
% C_M(\delta_{E})
%  

% Cf. Wang, Eqn. (A13)
c_CMdE = 0.0292;
% CMdEu(xvs) = c_CMdE;
CMdEu = c_CMdE;
CMdE0(xvs) = c_CMdE * ( - xvs(inda));
CMdE(xvus) = CMdE0 + CMdEu * xvus(inddE);
% DEBUGGING: Evaluate at trim
CMdE0e = double(subs(CMdE0,xvs,xve))

% ***********************
%       
% THRUST COEFFICIENT -- TERM k(x)
%  

% Do C_T AOA effects (=1) or not (=0)
doCTAOA = 0;

% Cf. Wang, Eqn. (A10)
c_k = 0.0105;
% c_k = 38 / S;
if doCTAOA 
    kxv(xvs) = c_k * (1 - 164 * (xvs(inda) - alpha_er)^2) * (1 + 17/M);
else
    kxv(xvs) = c_k * (1 + 17/M);
end
% DEBUGGING: Evaluate at trim
ke = double(subs(kxv,xvs,xve))

% ***********************
%       
% THRUST COEFFICIENT C_T
%  

% Cf. Wang, Eqn. (A10)
% NOTE: Used the case \delta_{T} < 1 only
CTu(xvs) = kxv * (1 + 0.15);
CT(xvs) = CTu * xvs(inddT);
% DEBUGGING: Evaluate at trim
CTe = double(subs(CT,xvs,xve))


% ***********************
%       
% DYNAMIC PRESSURE q_{\infty}
%  

qinf(xvs) = 0.5 * rho * xvs(indV)^2;
% DEBUGGING: Evaluate at trim
qinfe = double(subs(qinf,xvs,xve))


% ***********************
%       
% THRUST T
%  

% Cf. Wang, Eqn. (27)
Tu(xvs) = qinf * S * CTu;
T(xvs) = qinf * S * CT;
% DEBUGGING: Evaluate at trim
Te = double(subs(T,xvs,xve))


% ***********************
%       
% LIFT L
%  

% Cf. Wang, Eqn. (25)
L(xvs) = qinf * S * CL;
% DEBUGGING: Evaluate at trim
Le = double(subs(L,xvs,xve))

% Perturbation
L_nu = qinf * S * CL_nu;


% ***********************
%       
% LIFT L -- WITH ELEVATOR TERMS ADDED
%  

LdE0(xvs) = L;
LdEu(xvs) = qinf * S * CLdEu;
LdE(xvus) = formula(LdE0) + formula(LdEu) * xvus(inddE);

% DEBUGGING: Evaluate at trim
LdE0e = double(subs(LdE0,xvs,xve))
% DEBUGGING: Evaluate at trim
LdEue = double(subs(LdEu,xvs,xve))
% DEBUGGING: Evaluate at trim
LdEe = double(subs(LdE0,xvs,xve)) + double(subs(LdEu,xvs,xve))*ue_ws(2)
% LdEe = double(subs(L,xvus,[xve;ue]))

% Perturbation
LdE0_nu = L_nu;
LdE_nu(xvus) = formula(LdE0_nu) + formula(LdEu) * xvus(inddE);



% ***********************
%       
% DRAG D
%  

% Cf. Wang, Eqn. (26)
D(xvs) = qinf * S * CD;
% DEBUGGING: Evaluate at trim
De = double(subs(D,xvs,xve))


% ***********************
%       
% PITCHING MOMENT M_{yy}
%  

% Cf. Wang, Eqn. (29)
Myy0(xvs) = qinf * S * cbar * (CMalpha + CMdE0 + CMq);
Myyu(xvs) = qinf * S * cbar * CMdEu;
Myy(xvus) = formula(Myy0) + formula(Myyu) * xvus(inddE);
% DEBUGGING: Evaluate at trim
Myy0e = double(subs(Myy0,xvs,xve))
% DEBUGGING: Evaluate at trim
Myyue = double(subs(Myyu,xvs,xve))
% DEBUGGING: Evaluate at trim
Myye = double(subs(Myy0,xvs,xve)) + double(subs(Myyu,xvs,xve))*ue_ws(2)
% Myye = double(subs(Myy,xvus,[xve;ue]))


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE HSV MODEL -- DEFAULT PARAMETERS
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
%       
% PACK PARAMETERS
% 
% *************************************************************************


% ***********************
%       
% SYMBOLIC VARIABLES
%      

model.xs = xs;
model.xvs = xvs;
model.us = us; 
model.xvus = xvus;
model.indsz = indsz;
model.xvs0 = xvs0;

% Degree/radian conversions
model.D2R = D2R;
model.R2D = R2D;

% sin(\alpha), cos(\alpha)
model.sa = sa;
model.ca = ca;

% sin(\gamma), cos(\gamma)
model.sg = sg;
model.cg = cg;

% sin(\alpha+\gamma), cos(\alpha+\gamma)
model.sapg = sapg;
model.capg = capg;



% ***********************
%
% VEHICLE PARAMETERS
%

model.m = m;

model.g = g;
model.mu = mu;
model.RE = RE;
model.wref = wref;
model.mref = mref;
model.Iyy = Iyy;


model.S = S;
model.cbar = cbar;
% model.b = b;

model.k1 = k1;
model.k2 = k2;
model.k3 = k3;

% Index variables (i.e., where each state is in x)
model.indV = indV;
model.indg = indg;
model.indh = indh;
model.inda = inda;
model.indq = indq;
model.inddT = inddT;
model.indddT = indddT;
% model.indVI = indVI;
% model.indhI = indhI;

% Index variables (i.e., where each control is in [xv u])
model.inddTc = inddTc;
model.inddE = inddE;


% ***********************
%
% VEHICLE AERODYNAMIC FUNCTIONS
%

% Initial trim estimate
model.xve = xve;
model.ue_ws = ue_ws;

model.r = r;
model.rho = rho;
model.a = a;
model.M = M;

model.CL = CL;
model.CD = CD;

model.c_CMdE = c_CMdE;
model.CMdEu = CMdEu;
model.CMdE0 = CMdE0;
model.CMdE = CMdE;

model.kxv = kxv;
model.CT = CT;

model.qinf = qinf;

model.T = T;
model.Tu = Tu;

model.L = L;
model.D = D;

model.Myy0 = Myy0;
model.Myyu = Myyu;
model.Myy = Myy;

% ELEVATOR-INDUCED LIFT EFFECTS
model.LdE0 = LdE0;
model.LdEu = LdEu;
model.LdE = LdE;


% ***********************
%
% CALL INTIALIZATION FUNCTION
%

model = init_hsv_model(model);
trimconds_nom = model.trimconds;

% ***********************
%
% LINEARIZATION -- NOMINAL MODEL AT NOMINAL MODEL TRIM
%

lin_nom = linearize_hsv_model(model, trimconds_nom);
model.lin = lin_nom;


%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE HSV MODEL -- PERTURBED PARAMETERS
%
% *************************************************************************
% *************************************************************************
% *************************************************************************

% *************************************************************************
%       
% PACK PARAMETERS
% 
% *************************************************************************

% Initialize as the default, modify perturbed functions
model_nu = model;

% Store pertubation values
model_nu.nu = nu;

% Lift, drag
model_nu.CL = CL_nu;
model_nu.L = L_nu;

% ELEVATOR-INDUCED LIFT EFFECTS
model_nu.LdE0 = LdE0_nu;
% model_nu.LdEu = LdEu;
model_nu.LdE = LdE_nu;


% ***********************
%
% CALL INTIALIZATION FUNCTION
%

model_nu = init_hsv_model(model_nu);
trimconds_nu = model_nu.trimconds;

% ***********************
%
% LINEARIZATION -- PERTURBED MODEL AT PERTURBED MODEL TRIM
%

lin_nu = linearize_hsv_model(model_nu, trimconds_nu);
model_nu.lin = lin_nu;


% ***********************
%
% LINEARIZATION -- NOMINAL MODEL AT PERTURBED MODEL TRIM
%

lin_nom_atnu = linearize_hsv_model(model, trimconds_nu);
model.lin_atnu = lin_nom_atnu;

% ***********************
%
% LINEARIZATION -- PERTURBED MODEL AT NOMINAL MODEL TRIM
%

lin_nu_atnom = linearize_hsv_model(model_nu, trimconds_nom);
model_nu.lin_atnom = lin_nu_atnom;


% *************************************************************************
% *************************************************************************
%
% SAVE DATA
%
% *************************************************************************
% *************************************************************************


% Make directory to save data to
relpath_data = 'aircraft/hsv_wang_stengel_2000/data/'; 
mkdir(relpath_data)

% Initialize model struct
model_struct.model = model;
model_struct.model_nu = model_nu;

% Save data 
varname = 'model_struct';
filename = 'hsv_wang_stengel_2000_model';
save([relpath_data filename], varname);



%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% END MAIN
% 
% *************************************************************************
% *************************************************************************
% *************************************************************************

% Display complete
disp('*******************************************************************')
disp('*******************************************************************')
disp('*')
disp(['* MODEL INITIALIZATION COMPLETE'])
disp('*')
disp('*******************************************************************')
disp('*******************************************************************')




%%
% *************************************************************************
% *************************************************************************
% *************************************************************************
%
% INITIALIZE HSV MODEL FROM AERO DATA
%
% *************************************************************************
% *************************************************************************
% *************************************************************************


function model = init_hsv_model(model)


% ***********************
%       
% GLOBAL VARIABLES
%   

global sys;

% *************************************************************************
% *************************************************************************
%       
% UNPACK PARAMETERS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% SYMBOLIC VARIABLES
%      

xs = model.xs;
xvs = model.xvs;
us = model.us; 
xvus = model.xvus;
indsz = model.indsz;
xvs0 = model.xvs0;

% Degree/radian conversions
D2R = model.D2R;
R2D = model.R2D;

% sin(\alpha), cos(\alpha)
sa = model.sa;
ca = model.ca;

% sin(\gamma), cos(\gamma)
sg = model.sg;
cg = model.cg;

% sin(\alpha+\gamma), cos(\alpha+\gamma)
sapg = model.sapg;
capg = model.capg;



% ***********************
%
% VEHICLE PARAMETERS
%

% Initial trim estimate
xve = model.xve;
ue_ws = model.ue_ws;

m = model.m;

g = model.g;
mu = model.mu;
RE = model.RE;
wref = model.wref;
mref = model.mref;
Iyy = model.Iyy;


S = model.S;
cbar = model.cbar;
% b = model.b;

k1 = model.k1;
k2 = model.k2;
k3 = model.k3;

% Index variables (i.e., where each state is in x)
indV = model.indV;
indg = model.indg;
indh = model.indh;
inda = model.inda;
indq = model.indq;
inddT = model.inddT;
indddT = model.indddT;
% indVI = model.indVI;
% indhI = model.indhI;

% Index variables (i.e., where each control is in [xv u])
inddTc = model.inddTc;
inddE = model.inddE;


% ***********************
%
% VEHICLE AERODYNAMIC FUNCTIONS
%

r = model.r;
rho = model.rho;
a = model.a;
M = model.M;

CL = model.CL;
CD = model.CD;

c_CMdE = model.c_CMdE;
CMdEu = model.CMdEu;
CMdE0 = model.CMdE0;
CMdE = model.CMdE;

kxv = model.kxv;
CT = model.CT;

qinf = model.qinf;

T = model.T;
Tu = model.Tu;

L = model.L;
D = model.D;

Myy0 = model.Myy0;
Myyu = model.Myyu;
Myy = model.Myy;

% ELEVATOR-INDUCED LIFT EFFECTS
LdE0 = model.LdE0;
LdEu = model.LdEu;
LdE = model.LdE;

% *************************************************************************
% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS
%
% Cf. Wang, Stengel (2000), Eqns. (20)-(24), (35), (52)
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% FIRST DERIVATIVES
%    

% \dot{V}
% Cf. Wang, Eqn. (20)
dVu(xvs) = Tu*ca /mref;
dV(xvs) = (T*ca - D)/mref - mu*sg/r^2;

% \dot{\gamma}
% Cf. Wang, Eqn. (21)
dgu(xvs) = Tu*sa / (mref*xvs(indV));
dg(xvs) = (L + T*sa) / (mref*xvs(indV)) ...
    - ((mu-xvs(indV)^2*r)*cg) / (xvs(indV)*r^2);

% \dot{\gamma} -- ELEVATOR TERM \delta_{E}
dgudE(xvs) = LdEu / (mref*xvs(indV));

% % DEBUGGING: \dot{\gamma} terms
% dgr(xvs) = ((mu-xvs(indV)^2*r)*cg) / (xvs(indV)*r^2);
% dgre = double(subs(dgr,xvs,xve))

% \dot{h}
% Cf. Wang, Eqn. (22)
dh(xvs) = xvs(indV)*sg;

% \dot{\alpha}
% Cf. Wang, Eqn. (23)
dau(xvs) = - dgu;
da(xvs) = xvs(indq) - dg;

% \dot{q}
% Cf. Wang, Eqn. (24)
dqu(xvs) = Myyu / Iyy;
dq0(xvs) = Myy0 / Iyy;
% dq(xvs) = Myy / Iyy;

% \ddot{\delta_{T}}
% Cf. Wang, Eqn. (35)
dddT0(xvs) = k1 * xvs(indddT) + k2 * xvs(inddT);
% dddTc(xvs) = k3;
dddTc = k3;
% dddT(xvs) = dddT0 + dddTc * xvs(inddTc);

% \ddot{\alpha}
% Cf. Wang, Eqns. (23), (43), (45)
% dda0(xvs) = Myy0 / Iyy - ddg;
ddadE(xvs) = Myyu / Iyy;
% dda(xvs) = dq - ddg;


% *************************************************************************
% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS -- NO ACTUATOR DYNAMICS
%
% Cf. Wang, Stengel (2000), Eqns. (20)-(24), (35), (52)
%
% Throttle setting \delta_{T} is now a control, no longer a state. The
% state vector is
%
% x = [V, \gamma, h, \alpha, q] in R^{5}
% 
% *************************************************************************
% *************************************************************************


% \dot{V} -- f(x)
% Cf. Wang, Eqn. (20)
dV0x(xs) = subs(dV,xvs,xvs0);

% \dot{V} -- g(x)
% Cf. Wang, Eqn. (20)
dVux(xs) = subs(dVu,xvs,xvs0);

% \dot{\gamma} -- f(x)
% Cf. Wang, Eqn. (21)
dg0x(xs) = subs(dg,xvs,xvs0);

% \dot{\gamma} -- g(x)
% Cf. Wang, Eqn. (21)
dgux(xs) = subs(dgu,xvs,xvs0);

% \dot{\gamma} -- g(x) -- ELEVATOR TERM \delta_{E}
% Cf. Wang, Eqn. (21)
dgudEx(xs) = subs(dgudE,xvs,xvs0);

% \dot{h}
% Cf. Wang, Eqn. (22)
dhx(xs) = subs(dh,xvs,xvs0);

% \dot{\alpha} -- f(x)
% Cf. Wang, Eqn. (23)
da0x(xs) = subs(da,xvs,xvs0);

% \dot{\alpha} -- g(x)
% Cf. Wang, Eqn. (23)
daux(xs) = subs(dau,xvs,xvs0);

% \dot{q} -- f(x)
% Cf. Wang, Eqn. (24)
dq0x(xs) = subs(dq0,xvs,xvs0);

% \dot{q} -- g(x)
% Cf. Wang, Eqn. (24)
dqux(xs) = subs(dqu,xvs,xvs0);




% *************************************************************************
%
% SYSTEM DYNAMICAL EQUATIONS -- VECTOR FORM
%
% Cf. Wang, Stengel (2000), Eqns. (20)-(24), (35), (52)
% 
% *************************************************************************


% f(x) -- with actuator dynamics
fxvs(xvs) = [   dV
                dg
                dh
                da
                dq0
                xvs(indddT)
                dddT0       ];

% g(x) -- with actuator dynamics
gxvs(xvs) = [   0       0
                0       0
                0       0
                0       0
                0       ddadE
                0       0
                dddTc   0       ];

% g(x) -- with actuator dynamics -- WITH ELEVATOR LIFT EFFECTS
gxvdEs(xvs) = [ 0       0
                0       dgudE
                0       0
                0       -dgudE
                0       ddadE
                0       0
                dddTc   0       ];

% f(x) -- without actuator dynamics
fxs(xs) =   [   dV0x
                dg0x
                dhx
                da0x
                dq0x    ];


% g(x) -- without actuator dynamics
gxs(xs) =   [   dVux    0
                dgux    0
                0       0
                daux    0
                0       dqux    ];

% g(x) -- without actuator dynamics -- WITH ELEVATOR LIFT EFFECTS
gxdEs(xs) = [   dVux    0
                dgux    dgudEx
                0       0
                daux    -dgudEx
                0       dqux    ];


%%
% *************************************************************************
% *************************************************************************
%
% PARTIAL DERIVATIVES OF AERO FUNCTIONS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% THRUST T
%      

% Jacobian
dTdz = jacobian(T);
fdTdz = formula(dTdz);

% % Hessian
% d2Tdz2 = jacobian(dTdz);
% fd2Tdz2 = formula(d2Tdz2);

% Extract terms from Jacobian
T_V = fdTdz(indV);
T_a = fdTdz(inda);
T_dT = fdTdz(inddT);
T_h = fdTdz(indh);

% % Extract terms from Hessian
% T_VV = fd2Tdz2(indV,indV);
% T_Va = fd2Tdz2(indV,inda);
% T_VdT = fd2Tdz2(indV,inddT);
% T_Vh = fd2Tdz2(indV,indh);
% T_aa = fd2Tdz2(inda,inda);
% T_adT = fd2Tdz2(inda,inddT);
% T_ah = fd2Tdz2(inda,indh);
% T_dTdT = fd2Tdz2(inddT,inddT);
% T_dTh = fd2Tdz2(inddT,indh);
% T_hh = fd2Tdz2(indh,indh);


% ***********************
%       
% LIFT L
%      

% Jacobian
dLdz = jacobian(L);
fdLdz = formula(dLdz);

% % Hessian
% d2Ldz2 = jacobian(dLdz);
% fd2Ldz2 = formula(d2Ldz2);

% Extract terms from Jacobian
L_V = fdLdz(indV);
L_a = fdLdz(inda);
L_h = fdLdz(indh);

% % Extract terms from Hessian
% L_VV = fd2Ldz2(indV,indV);
% L_Va = fd2Ldz2(indV,inda);
% L_Vh = fd2Ldz2(indV,indh);
% L_aa = fd2Ldz2(inda,inda);
% L_ah = fd2Ldz2(inda,indh);
% L_hh = fd2Ldz2(indh,indh);

% ***********************
%       
% DRAG D
%      

% Jacobian
dDdz = jacobian(D);
fdDdz = formula(dDdz);

% % Hessian
% d2Ddz2 = jacobian(dDdz);
% fd2Ddz2 = formula(d2Ddz2);

% Extract terms from Jacobian
D_V = fdDdz(indV);
D_a = fdDdz(inda);
D_h = fdDdz(indh);

% % Extract terms from Hessian
% D_VV = fd2Ddz2(indV,indV);
% D_Va = fd2Ddz2(indV,inda);
% D_Vh = fd2Ddz2(indV,indh);
% D_aa = fd2Ddz2(inda,inda);
% D_ah = fd2Ddz2(inda,indh);
% D_hh = fd2Ddz2(indh,indh);


% ***********************
%       
% PITCHING MOMENT M_{yy}|_{u=0}
%     

% Jacobian
dMyy0dxv = jacobian(Myy0);
fdMyy0dxv = formula(dMyy0dxv);

Myy0_V = fdMyy0dxv(indV);
Myy0_h = fdMyy0dxv(indh);
Myy0_a = fdMyy0dxv(inda);
Myy0_q = fdMyy0dxv(indq);


%%
% *************************************************************************
% *************************************************************************
%
% CREATE INLINE FUNCTIONS
% 
% *************************************************************************
% *************************************************************************

% f(x) -- with actuator dynamics
fil = matlabFunction(fxvs, 'vars', {xvs});

% g(x) -- with actuator dynamics
gil = matlabFunction(gxvs, 'vars', {xvs});

% g(x) -- with actuator dynamics -- WITH ELEVATOR LIFT EFFECTS
gildE = matlabFunction(gxvdEs, 'vars', {xvs});

% f(x) -- without actuator dynamics
fx = matlabFunction(fxs, 'vars', {xs});
sys.model.fx = fx;

% g(x) -- without actuator dynamics
gx = matlabFunction(gxs, 'vars', {xs});
sys.model.gx = gx;

% g(x) -- without actuator dynamics -- WITH ELEVATOR LIFT EFFECTS
gxdE = matlabFunction(gxdEs, 'vars', {xs});
sys.model.gxdE = gxdE;

% Dynamic pressure q_{\infty}
qinfil = matlabFunction(qinf, 'vars', {xvs});

% Air density \rho
rhoil = matlabFunction(rho, 'vars', {xvs});

% Mach number M
Mil = matlabFunction(M, 'vars', {xvs});

% L(x), D(x), T(x)
Lil = matlabFunction(L, 'vars', {xvs});
Dil = matlabFunction(D, 'vars', {xvs});
Til = matlabFunction(T, 'vars', {xvs});

% Lift L -- WITH ELEVATOR LIFT EFFECTS
LdEil = matlabFunction(LdE, 'vars', {xvus});

% M_{yy}
Myyil = matlabFunction(Myy, 'vars', {xvus});

% r(h)
ril = matlabFunction(r, 'vars', {xvus});

% k(x) used for thrust calculation
kil = matlabFunction(kxv, 'vars', {xvs});

% L(x) -- partial derivatives
L_Vil = matlabFunction(L_V, 'vars', {xvs});
L_ail = matlabFunction(L_a, 'vars', {xvs});
L_hil = matlabFunction(L_h, 'vars', {xvs});

% D(x) -- partial derivatives
D_Vil = matlabFunction(D_V, 'vars', {xvs});
D_ail = matlabFunction(D_a, 'vars', {xvs});
D_hil = matlabFunction(D_h, 'vars', {xvs});

% T(x) -- partial derivatives
T_Vil = matlabFunction(T_V, 'vars', {xvs});
T_ail = matlabFunction(T_a, 'vars', {xvs});
T_hil = matlabFunction(T_h, 'vars', {xvs});

% M_{yy}|_{u=0} -- partial derivatives
Myy0_Vil = matlabFunction(Myy0_V, 'vars', {xvs});
Myy0_hil = matlabFunction(Myy0_h, 'vars', {xvs});
Myy0_ail = matlabFunction(Myy0_a, 'vars', {xvs});
Myy0_qil = matlabFunction(Myy0_q, 'vars', {xvs});


%%
% *************************************************************************
% *************************************************************************
%
% CALCULATE TRIM -- WITHOUT LIFT ELEVATOR EFFECTS
%
% See trimcost_3dof.m
%
% *************************************************************************
% *************************************************************************

% Initialize trim parameters
M0 = 15;                            % M = 15 at trim
a0 = double(subs(a,xvs,xve));       % Solve for speed of sound at h_0
V0 = M0*a0;                         % V = M * a
gamma0 = 0;                         % Level flight trim condition
h0 = 110e3;                         % h_0 = 110 kft
% Store trim conditions
trimconds.V0 = V0;               
trimconds.gamma0 = gamma0;           
trimconds.h0 = h0;              
sys.model.trimconds = trimconds;

% Initial guess at trim values. From Wang, Stengel (2000)
% optparam = [  dT      Throttle setting
%               dE      Elevator deflection (rad)
%               alpha   AOA (rad)                   ]
initparam = [   ue_ws(1)
                ue_ws(2)
                xve(inda) ];

% fminsearch options
options = optimset('TolX',1e-16, 'TolFun',1e-16, 'MaxFunEvals',1000);
% options = optimset('TolX',1e-6, 'TolFun',1e-10);
% options = optimset('TolX',1e-4, 'TolFun',1e-4);

% Peform trim solve
[optparam, J, exitflag, output] = ...
    fminsearch('trimcost_3dof', initparam, options);

% Get trim values
dT0 = optparam(1);
dE0 = optparam(2);
alpha0 = optparam(3);

disp('***********************')
disp('*')
disp('* TRIM CALCULATION -- WITHOUT LIFT ELEVATOR EFFECTS')
disp('*')
disp('***********************')
disp(' ')

% Final trim cost, exit flag
disp(['Trim Cost = ' num2str(J)])
disp(['Exit Flag = ' num2str(exitflag)])

% Trim settings -- throttle d_T, elevator d_E, AOA \alpha
disp(['Trim Throttle            d_T =      ' num2str(dT0)])
disp(['Trim Elevator Deflection d_E =      ' num2str(dE0) ' rad = ' ...
    num2str(R2D*dE0) ' deg'])
disp(['Trim AOA                 alpha =    ' num2str(alpha0) ' rad = ' ...
    num2str(R2D*alpha0) ' deg'])

% Trim state x_0
disp(['Trim State x_0 = [V_0, \gamma_0, h_0, \alpha_0, q_0]^T = '])
xe = [  V0
        gamma0
        h0
        alpha0
        0               ]
% xve = [xe ; dT0 ; 0]

% Trim control x_0
disp(['Trim Control u_0 = [d_{T,0}, d_{E,0}]^T = '])
ue = [  dT0
        dE0     ]
				
% Evaluate system dynamics
f_xe = fx(xe);
g_xe = gx(xe);
xdot = f_xe + g_xe * ue;

disp(['State Derivatives Evaluated at Trim: ' ...
    '\dot{x} = f(x_0) + g(x_0) * u_0 ='])

xdot

disp(['||\dot{x} = f(x_0) + g(x_0) * u_0 || ='])

norm(xdot)

% Store equilibrium calculations
trimconds.xe = xe;
trimconds.ue = ue;


%%
% *************************************************************************
% *************************************************************************
%
% CALCULATE TRIM -- WITH LIFT ELEVATOR EFFECTS
%
% See trimcost_3dof.m
%
% *************************************************************************
% *************************************************************************

% Temporarily set "g(x)" field to reflect elevator effects for trim
% calculation
sys.model.gx = gxdE;

% Peform trim solve
[optparam, J, exitflag, output] = ...
    fminsearch('trimcost_3dof', initparam, options);

% Reset "g(x)" field to default Wang, Stengel model
sys.model.gx = gx;

% Get trim values
dT0_dE = optparam(1);
dE0_dE = optparam(2);
alpha0_dE = optparam(3);

disp('***********************')
disp('*')
disp('* TRIM CALCULATION -- WITH LIFT ELEVATOR EFFECTS')
disp('*')
disp('***********************')
disp(' ')

% Final trim cost, exit flag
disp(['Trim Cost = ' num2str(J)])
disp(['Exit Flag = ' num2str(exitflag)])

% Trim settings -- throttle d_T, elevator d_E, AOA \alpha
disp(['Trim Throttle            d_T =      ' num2str(dT0_dE)])
disp(['Trim Elevator Deflection d_E =      ' num2str(dE0_dE) ' rad = ' ...
    num2str(R2D*dE0_dE) ' deg'])
disp(['Trim AOA                 alpha =    ' num2str(alpha0_dE) ...
    ' rad = ' num2str(R2D*alpha0_dE) ' deg'])

% Trim state x_0
disp(['Trim State x_0 = [V_0, \gamma_0, h_0, \alpha_0, q_0]^T = '])
xe_dE = [  V0
        gamma0
        h0
        alpha0_dE
        0               ]

% Trim control x_0
disp(['Trim Control u_0 = [d_{T,0}, d_{E,0}]^T = '])
ue_dE = [  dT0_dE
        dE0_dE     ]
				
% Evaluate system dynamics
f_xe_dE = fx(xe_dE);
g_xe_dE = gx(xe_dE);
xdot_dE = f_xe_dE + g_xe_dE * ue_dE;

disp(['State Derivatives Evaluated at Trim: ' ...
    '\dot{x} = f(x_0) + g(x_0) * u_0 ='])

xdot_dE

disp(['||\dot{x} = f(x_0) + g(x_0) * u_0 || ='])

norm(xdot_dE)

% Store equilibrium calculations
trimconds.xe_dE = xe_dE;
trimconds.ue_dE = ue_dE;


%%
% *************************************************************************
% *************************************************************************
%
% SYMBOLIC LINEARIZATION
% 
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% LINEARIZATION -- FROM HAND DERIVATION -- WITHOUT ACTUATOR DYNAMICS
% 
% *************************************************************************

% ***********************
%       
% df_1 / dx
%    
% % a11(xvs) = (T_V * ca - D_V) / mref;
% % a12(xvs) = - mu * cg / r^2;
% % a13(xvs) = (T_h * ca - D_h) / mref + 2 * mu * sg / r^3;
% % a14(xvs) = (T_a * ca - T * sa - D_a) / mref;
% % % a15 = 0;
% % a16 = T_dT * ca / mref;
% % % a17 = 0;

a11(xvs) = (- D_V) / mref;
a12(xvs) = - mu * cg / r^2;
a13(xvs) = (- D_h) / mref + 2 * mu * sg / r^3;
a14(xvs) = (- D_a) / mref;
% a15 = 0;
a16 = T_dT * ca / mref;
% a17 = 0;

% ***********************
%       
% df_2 / dx
%   
% % a21(xvs) = (L_V + T_V * sa) / (mref*xvs(indV)) ...
% %     - (L + T * sa) / (mref*xvs(indV)^2) + 2 * cg / r ...
% %     + (mu - xvs(indV)^2 * r) * cg / (xvs(indV)*r)^2;
% % a22(xvs) = (mu - xvs(indV)^2 * r) * sg / (xvs(indV) * r^2);
% % a23(xvs) = (L_h + T_h * sa) / (mref*xvs(indV)) + xvs(indV) * cg / r^2 ...
% %     + 2 * (mu - xvs(indV)^2 * r) * cg / (xvs(indV) * r^3);
% % a24(xvs) = (L_a + T_a * sa + T * ca) / (mref*xvs(indV));
% % % a25 = 0;
% % a26 = T_dT * sa / (mref*xvs(indV));
% % % a27 = 0;

a21(xvs) = (L_V ) / (mref*xvs(indV)) ...
    - (L ) / (mref*xvs(indV)^2) + 2 * cg / r ...
    + (mu - xvs(indV)^2 * r) * cg / (xvs(indV)*r)^2;
a22(xvs) = (mu - xvs(indV)^2 * r) * sg / (xvs(indV) * r^2);
a23(xvs) = (L_h ) / (mref*xvs(indV)) + xvs(indV) * cg / r^2 ...
    + 2 * (mu - xvs(indV)^2 * r) * cg / (xvs(indV) * r^3);
a24(xvs) = (L_a) / (mref*xvs(indV));
% a25 = 0;
a26 = T_dT * sa / (mref*xvs(indV));
% a27 = 0;


% ***********************
%       
% df_3 / dx
%   
a31 = sg;
a32 = xvs(indV) * cg;
% a33 = 0;
% a34 = 0;
% a35 = 0;
% a36 = 0;
% a37 = 0;

% ***********************
%       
% df_4 / dx
%   
% See below

% ***********************
%       
% df_5 / dx
%   

a51 = Myy0_V / Iyy;
% a52 = 0;
a53 = Myy0_h / Iyy;
a54 = Myy0_a / Iyy;
a55 = Myy0_q / Iyy;
% a56 = 0
% a57 = 0;


% ***********************
%       
% "B" MATRIX -- ADDING IN ELEVATOR-INDUCED LIFT EFFECTS 
%   
b22 = dgudE;


% ***********************
%       
% "A", "B", "C", "D" MATRIX FOR STANDARD 3-DOF MODEL
%   

% d f / d x
dfdx_sym = [  a11     a12     a13     a14     0
            a21     a22     a23     a24     0
            a31     a32     0       0       0
            -a21    -a22    -a23    -a24    1
            a51     0       a53     a54     a55     ];
dfdx_sym(xs) = subs(dfdx_sym,xvs,xvs0);
dfdx_sym = formula(dfdx_sym);

% "B" MATRIX -- WITHOUT ELEVATOR-INDUCED LIFT EFFECTS
% g_1(x): d_T
g1s =  [    a16    
            a26    
            0     
            -a26  
            0        ];
g1s(xs) = subs(g1s,xvs,xvs0);
g1s(xs) = formula(g1s);
% g_2(x): d_E
g2s =  [    0   
            0    
            0     
            0  
            ddadE        ];
g2s(xs) = subs(g2s,xvs,xvs0);
g2s(xs) = formula(g2s);

Bp_sym(xs) = formula([g1s g2s]);


% "B" MATRIX -- WITH ELEVATOR-INDUCED LIFT EFFECTS
% g_2(x): d_E (with elevator-lift effects)
g2sdE = [   0   
            b22    
            0     
            -b22  
            ddadE        ];
g2sdE(xs) = subs(g2sdE,xvs,xvs0);
g2sdE(xs) = formula(g2sdE);


BpdE_sym(xs) = formula([g1s g2sdE]);

% d g_j(x) / d x 
dg1dxs = formula(jacobian(g1s));
dg2dxs = formula(jacobian(g2s));
dg2dxsdE = formula(jacobian(g2sdE));


%%
% *************************************************************************
% *************************************************************************
%
% STORE MODEL PARAMETERS
%
% *************************************************************************
% *************************************************************************


% ***********************
%
% SYSTEM DYNAMICAL EQUATIONS
%

% With actuator dynamics
model.fxvs = fxvs;
model.gxvs = gxvs;

% Without actuator dynamics
model.fxs = fxs;
model.gxs = gxs;

% g(x) -- without actuator dynamics -- WITH ELEVATOR LIFT EFFECTS
model.gxdE = gxdE;

% ***********************
%
% INLINE FUNCTIONS
%

% With actuator dynamics
model.fil = fil;
model.gil = gil;
model.gildE = gildE;    % WITH ELEVATOR-LIFT EFFECTS

% Without actuator dynamics
model.fx = fx;
model.gx = gx;

% Dynamic pressure q_{\infty}
model.qinfil = qinfil;

% Air density \rho
model.rhoil = rhoil;

% Mach number M
model.Mil = Mil;

% L(x), D(x), T(x)
model.Lil = Lil;
model.Dil = Dil;
model.Til = Til;

% Lift L -- WITH ELEVATOR LIFT EFFECTS
model.LdEil = LdEil;

% M_{yy}
model.Myyil = Myyil;

% r(h)
model.ril = ril;

% k(x_v) used for thrust calculation
model.kil = kil;

% L(x) -- partial derivatives
model.L_Vil = L_Vil;
model.L_ail = L_ail;
model.L_hil = L_hil;

% D(x) -- partial derivatives
model.D_Vil = D_Vil;
model.D_ail = D_ail;
model.D_hil = D_hil;

% T(x) -- partial derivatives
model.T_Vil = T_Vil;
model.T_ail = T_ail;
model.T_hil = T_hil;

% M_{yy}|_{u=0} -- partial derivatives
model.Myy0_Vil = Myy0_Vil;
model.Myy0_hil = Myy0_hil;
model.Myy0_ail = Myy0_ail;
model.Myy0_qil = Myy0_qil;

% ELEVATOR-INDUCED LIFT EFFECTS
model.dgudE = dgudE;


% ***********************
%
% VEHICLE TRIM
%

model.trimconds = trimconds;


% ***********************
%
% SYMBOLIC LINEARIZATION TERMS
%

model.dfdx_sym = dfdx_sym;
model.g1s = g1s;
model.g2s = g2s;
model.Bp_sym = Bp_sym;
model.g2sdE = g2sdE;
model.BpdE_sym = BpdE_sym;
model.dg1dxs = dg1dxs;
model.dg2dxs = dg2dxs;
model.dg2dxsdE = dg2dxsdE;

%%
% *************************************************************************
% *************************************************************************
%
% LINEARIZE SYSTEM
% 
% *************************************************************************
% *************************************************************************

function lin = linearize_hsv_model(model, trimconds)


% ***********************
%       
% GLOBAL VARIABLES
%   

global sys;

% Number of inputs
m = 2;

% *************************************************************************
% *************************************************************************
%       
% UNPACK PARAMETERS
% 
% *************************************************************************
% *************************************************************************

% ***********************
%       
% SYMBOLIC VARIABLES
%      

xs = model.xs;
xvs = model.xvs;
us = model.us; 
xvus = model.xvus;
indsz = model.indsz;
xvs0 = model.xvs0;

% Degree/radian conversions
D2R = model.D2R;
R2D = model.R2D;

% sin(\alpha), cos(\alpha)
sa = model.sa;
ca = model.ca;

% sin(\gamma), cos(\gamma)
sg = model.sg;
cg = model.cg;

% sin(\alpha+\gamma), cos(\alpha+\gamma)
sapg = model.sapg;
capg = model.capg;

% Index variables (i.e., where each state is in x)
indV = model.indV;
indg = model.indg;
indh = model.indh;
inda = model.inda;
indq = model.indq;
inddT = model.inddT;
indddT = model.indddT;
% indVI = model.indVI;
% indhI = model.indhI;

% Index variables (i.e., where each control is in [xv u])
inddTc = model.inddTc;
inddE = model.inddE;


% ***********************
%
% SYMBOLIC LINEARIZATION TERMS
%

dfdx_sym = model.dfdx_sym;
g1s = model.g1s;
g2s = model.g2s;
Bp_sym = model.Bp_sym;
g2sdE = model.g2sdE;
BpdE_sym = model.BpdE_sym;
dg1dxs = model.dg1dxs;
dg2dxs = model.dg2dxs;
dg2dxsdE = model.dg2dxsdE;

% ***********************
%
% VEHICLE TRIM
%

xe = trimconds.xe;
ue = trimconds.ue;
xe_dE = trimconds.xe_dE;
ue_dE = trimconds.ue_dE;



% d {g(x) u} / d x 
dgudx = double(subs(dg1dxs, xs, xe)) * ue(1) ...
    + double(subs(dg2dxs, xs, xe)) * ue(2);
dgudxdE = double(subs(dg1dxs, xs, xe_dE)) * ue_dE(1) ...
    + double(subs(dg2dxsdE, xs, xe_dE)) * ue_dE(2);

% Outputs: [V h]
Cvh = [    1   0   0   0   0
            0   0   1   0   0   ];

% Outputs y = [V, \gamma]^T
Cvg = [eye(2) zeros(2,3)];

Dp = zeros(m);

% Evaluate numerically at trim
Ap = double(subs(dfdx_sym, xs, xe)) + dgudx;
Bp = double(subs(Bp_sym, xs, xe));

% Evaluate numerically at trim -- WITH ELEVATOR-LIFT EFFECTS
ApdE = double(subs(dfdx_sym, xs, xe_dE)) + dgudxdE;
BpdE = double(subs(BpdE_sym, xs, xe_dE));

% Plant y = [V, h]^T
Pvh = ss(Ap,Bp,Cvh,Dp);

% Plant y = [V, \gamma]^T
Pvg = ss(Ap,Bp,Cvg,Dp);

% Plant y = [V, h]^T -- WITH ELEVATOR-LIFT EFFECTS
PdEvh = ss(ApdE,BpdE,Cvh,Dp);

% Plant y = [V, \gamma]^T -- WITH ELEVATOR-LIFT EFFECTS
PdEvg = ss(ApdE,BpdE,Cvg,Dp);

% Plant y = x
Px = ss(Ap,Bp,eye(5),zeros(5,2));

% Plant y = x -- WITH ELEVATOR-LIFT EFFECTS
PdEx = ss(ApdE,BpdE,eye(5),zeros(5,2));



%%
% *************************************************************************
% *************************************************************************
%
% FORM DESIGN PLANT FOR PI-PD INNER-OUTER DESIGN
%
% *************************************************************************
% *************************************************************************

% *************************************************************************
%
% FORM DESIGN PLANT
%
% Given an original plant of the form P = (A, B, C, D), we would like to
% form a design plant (appropriately scaled) such that the state vector
% assumes the form
%
% x' = [    y
%           x_r     ]
%
% Where y \in R^m are the outputs to be tracked, and x_r \in R^{n-m} forms
% the rest of the state.
%
% Introduce the coordinate transformations:
%
%  u' = S_u * u
%  x' = S_x * x
%  y' = S_y * y
%
%
% New Control Variables
%     u' = [    d_T
%               d_E  (deg)]
%
%
% New State Variables
%       x' =   [    V (ft/s)
%                   \gamma (deg)
%                   -----------
%                   \theta (deg)
%                   q (deg/s)       ]
%
%            = [    y
%                   x_r             ]
%    
% New Output Variables
%       y' =   [    V (ft/s)
%                   \gamma (deg)    ] 
%          = y
%
% I.e., we have permuted the FPA \gamma and altitude h, we have converted
% all angular states/controls from rad to deg, and we have kept the outputs
% unchanged (since the outputs are both transaltional in nature, which we
% have not scaled).
%

% Remove height mode from original matrices
indsnoh = [(1:indh-1)'; (indh+1:5)'];
Apnoh = ApdE(indsnoh,indsnoh);
Bpnoh = BpdE(indsnoh,:);
Cvgnoh = Cvg(:,indsnoh);
Dpnoh = Dp;
Pnoh = ss(Apnoh,Bpnoh,Cvgnoh,Dpnoh);

% Velocity scaling factor for transformation
% Vscl = 1;
Vscl = 1/1000;

% Coordinate transformations
sud = diag([1 R2D]);
sxd = [ Vscl 0   0   0   
        0   R2D 0   0   
        0   R2D R2D 0 
        0   0   0   R2D        ];
syd = diag([Vscl R2D]);

% Coordinate transformation -- with h
sxdh = [ Vscl 0   0   0  0
        0   R2D 0   0   0  
        0   R2D 0   R2D 0 
        0   0   0   0   R2D
        0   0   1   0   0       ];


% Scaled linear dynamics
Ad = sxd*Apnoh*inv(sxd);
Bd = sxd*Bpnoh*inv(sud);
Cd = syd*Cvgnoh*inv(sxd);
Dd = syd*Dpnoh*inv(sud);

% Scaled design plant
Pd = ss(Ad,Bd,Cd,Dd);

% Scaled linear dynamics -- with h
Adh = sxdh*ApdE*inv(sxdh);
Bdh = sxdh*BpdE*inv(sud);
Cdh = syd*Cvg*inv(sxdh);
Ddh = syd*Dp*inv(sud);

% Scaled design plant -- with h
Pdh = ss(Adh,Bdh,Cdh,Ddh);

% *************************************************************************
%
% FORM PLANT FOR DIRL
%
% State: x = [  x_1
%               x_2
%               x_3     ]
%
%       x_1 = [ z_V (kft/s -s)
%               V   (kft/s)     ]
%
%       x_2 = [ z_\gamma    (deg-s)
%               \gamma      (deg)
%               \theta      (deg)
%               q           (deg/s) ]
%               
%       x_3 =   h   (ft)         
%
% *************************************************************************

% Augmented plant
% x = [z^T x_p^T]^T
Aaug = [    zeros(m)   Cdh
            zeros(5,m) Adh  ];
Baug = [    zeros(m,m)
            Bdh         ];

% DIRL state transformation
Sxirl = [   1 0 0       0 0 0 0
            0 0 1       0 0 0 0
            0 1 0       0 0 0 0
            zeros(4,3)  eye(4)  ];

% DIRL state-space matrices
Airl = Sxirl * Aaug * inv(Sxirl);
Birl = Sxirl * Baug;

% *************************************************************************
%
% FORM SIMPLIFIED DECOUPLED DESIGN PLANT FOR INNER-OUTER LOOP DESIGN
%
% *************************************************************************

% Whole system
Adio = Ad;
Adio(1,:) = 0;
Adio(:,1) = 0;
Adio(4,4) = 0;

Bdio = Bd;
Bdio(2,1) = 0;

Cdio = Cd;
Ddio = Dd;

% \delta_{T} -> V subsystem
AdTV = Adio(1,1);
BdTV = Bdio(1,1);
CdTV = 1;
DdTV = 0;

% \delta_{T} -> V subsystem -- exact A_{11}
Ad11 = Ad(1,1);
Bd11 = BdTV;
Cd11 = CdTV;
Dd11 = DdTV;

% \delta_{E} -> \gamma subsystem
AdEg = Adio(2:4,2:4);
BdEg = Bdio(2:4,2);
CdEg = [1 0 0];
DdEg = 0;

% \delta_{E} -> \gamma subsystem -- exact A_{22}
Ad22 = Ad(2:4,2:4);
Bd22 = BdEg;
Cd22 = CdEg;
Dd22 = DdEg;


% Multivariable design plant
Pdio = ss(Adio,Bdio,Cdio,Ddio);

% Plant \delta_{T} -> V
PdTV = ss(AdTV,BdTV,CdTV,DdTV);
% zpk(PdTV)

% Plant \delta_{T} -> V -- exact A_{11}
Pd11 = ss(Ad11,Bd11,Cd11,Dd11);

% Plant \delta_{E} -> \gamma
PdEg = ss(AdEg,BdEg,CdEg,DdEg);
% zpk(PdEg)

% Plant \delta_{E} -> \gamma -- exact A_{22}
Pd22 = ss(Ad22,Bd22,Cd22,Dd22);

% Plant \delta_{E} -> \theta
PdEth = ss(Adio,Bdio(:,2),[0 0 1 0],0);
% zpk(PdEth)

% Simplified linear dynamics -- re-scaled back to original
Apnohio = inv(sxd)*Adio*sxd; 
Bpnohio = inv(sxd)*Bpnoh*sud;
Cpnohio = inv(syd)*Cvgnoh*sxd;
Dpnohio = inv(syd)*Dpnoh*sud;



%%
% *************************************************************************
% *************************************************************************
%
% STORE MODEL PARAMETERS
%
% *************************************************************************
% *************************************************************************

% ***********************
%
% LINEARIZATION TERMS
%

% A, B, C, D matrix -- nominal MP model
lin.Ap = Ap;
lin.Bp = Bp;
lin.Cvh = Cvh;
lin.Cvg = Cvg;
lin.Dp = Dp;

% Plant ss objects -- nominal MP model
lin.Pvh = Pvh;          % Plant y = [V, h]^T
lin.Pvg = Pvg;          % Plant y = [V, \gamma]^T
lin.Px = Px;            % Plant y = x

% A, B, C, D matrix -- WITH ELEVATOR-LIFT EFFECTS
lin.ApdE = ApdE;
lin.BpdE = BpdE;

% Plant ss objects -- WITH ELEVATOR-LIFT EFFECTS
lin.PdEvh = PdEvh;      % Plant y = [V, h]^T
lin.PdEvg = PdEvg;      % Plant y = [V, \gamma]^T
lin.PdEx = PdEx;        % Plant y = x


% ***********************
%
% LINEARIZATION TERMS -- DESIGN PLANT FOR PD-PI INNER-OUTER
%

io.sud = sud;
io.sxd = sxd;
io.sxdh = sxdh;
io.syd = syd;

io.Apnoh = Apnoh;
io.Bpnoh = Bpnoh;
io.Cvgnoh = Cvgnoh;
io.Dpnoh = Dpnoh;
io.Pnoh = Pnoh;

io.Ad = Ad;
io.Bd = Bd;
io.Cd = Cd;
io.Dd = Dd;
io.Pd = Pd;

% Scaled linear dynamics -- with h
io.Adh = Adh;
io.Bdh = Bdh;
io.Cdh = Cdh;
io.Ddh = Ddh;

% IRL linear dynamics
io.Airl = Airl;
io.Birl = Birl;

% Scaled design plant -- with h
io.Pdh = Pdh;

io.Adio = Adio;
io.Bdio = Bdio;
io.Cdio = Cdio;
io.Ddio = Ddio;
io.Pdio = Pdio;

% \delta_{T} -> V subsystem
io.AdTV = AdTV;
io.BdTV = BdTV;
io.CdTV = CdTV;
io.DdTV = DdTV;
io.PdTV = PdTV;

% \delta_{T} -> V subsystem -- with exact A_{11}
io.Ad11 = Ad11;
io.Bd11 = Bd11;
io.Cd11 = Cd11;
io.Dd11 = Dd11;
io.Pd11 = Pd11;

% \delta_{E} -> \gamma subsystem
io.AdEg = AdEg;
io.BdEg = BdEg;
io.CdEg = CdEg;
io.DdEg = DdEg;
io.PdEg = PdEg;

% \delta_{E} -> \gamma subsystem -- with exact A_{22}
io.Ad22 = Ad22;
io.Bd22 = Bd22;
io.Cd22 = Cd22;
io.Dd22 = Dd22;
io.Pd22 = Pd22;


io.PdEth = PdEth;

% Simplified linear dynamics -- re-scaled back to original
io.Apnohio = Apnohio; 
io.Bpnohio = Bpnohio;
io.Cpnohio = Cpnohio;
io.Dpnohio = Dpnohio;


% Store all inner/outer terms
lin.io = io;


% Store linearization params
model.lin = lin;




