%% Filter Settings
[Nu, Du] = Gfgen(Gf);

FILT.Nu = Nu;
FILT.Du = Du; 

% Gz is Identity
FILT.Nz = eye(lz);
FILT.Dz = 0*FILT.Nz;

% Gphi
FILT.Dphi = FILT.Du;
FILT.Nphi = [zeros(lz,lu) FILT.Nu];

%% Controller Settings

FLAG.RegInit  = 0; % Compute regressor before all data is available
FLAG.FiltInit = 0; % Compute filter before all data is available
FLAG.ContInit = 0; % Compute controller before all data is available
FLAG.k_0 = k0;

% Controller Type
FLAG.Sparse     = sparseflag;
FLAG.GcStab     = gcstab;
FLAG.uFIR       = firflag;
FLAG.uZERO      = 0;
FLAG.Prop       = 0;
FLAG.Int        = 0;
FLAG.IntProp    = 0;

% Cost function type
FLAG.JInst = 0;

% Regression Variable
FLAG.RegZ = 0;

% Forgetting Factor
FLAG.lambda = 1; % Must be <= 1

% Constant Weights
FLAG.Rz     = 1*eye(lz); % Performance penalty
% FLAG.Rz     = [1e-10 0; 0 1];
FLAG.Rf     = Ruf*eye(lz); % Filtered Control penalty
FLAG.Ru     = Ru*eye(lu); % Control penalty

FLAG.alpha = 1;
FLAG.beta = 0;

% Compute regressor vector size
if isfield(FLAG, 'RegZ') && FLAG.RegZ
    lv = lz;
else
    lv = ly;
end

%% Compute controller order and size of control parameter
if FLAG.uZERO
    ltheta  = lu*ly;
    lphi    = ly;
elseif FLAG.uFIR
    if FLAG.Prop
        ltheta  = lu*(Nc+1)*ly;
        lphi    = ly*(Nc+1);
    else
        ltheta  = Nc*(lu*ly);
        lphi    = ly*Nc;
    end
else        % For Proper (instead of strictly proper) controller
    if FLAG.Sparse    
        ltheta  = Nc*(lu*ly+1);
        lphi    = (lu+ly)*Nc;
    elseif FLAG.GcStab
        ltheta = 2*Nc;
        lphi = (lu+ly)*Nc;
    elseif FLAG.Prop
        ltheta  = lu*(Nc*lu +(Nc+1)*ly);
        lphi    = lu*Nc+ly*(Nc+1);
    else
        ltheta  = Nc*(lu*(ly+lu));
        lphi    = (lu+ly)*Nc;
    end
end

if FLAG.Int % Integrator Size
    ltheta = ltheta+lu*lz;
    lphi = lphi+lz;
end

FLAG.KF_A = eye(ltheta);
% FLAG.KF_Q = 1e3*eye(ltheta);
FLAG.KF_Q = kfq*eye(ltheta);

%% Specify More Initial conditions
FLAG.Rtheta = Rt*eye(ltheta); % Transient Penalty
FLAG.Rdelta = Rd*eye(ltheta); % Delta t
%Theta Penalty

% Initial Parameters
FLAG.theta_0 = zeros(ltheta,1);


%% Memory allocation
x = zeros(lx, steps);
xol = zeros(lx, steps);

y = zeros(ly,steps);
yol = zeros(ly,steps);

z = zeros(lz,steps);
zol = zeros(lz,steps);

u = zeros(lu,steps);

z_hat = zeros(lz,steps);

phi_filt = zeros(lz,ltheta, steps);
phi = zeros(lphi,steps);
uf = zeros(lz,steps);
%P = zeros(ltheta, ltheta,steps);
theta = zeros(ltheta,steps);

%% Init
x(:,1) = 0*ones(lx,1);
u(:,1) = zeros(lu,1);
y(:,1) = C*x(:,1) + D2*wnoise(:,1);
z(:,1) = E1*x(:,1) - r;

xol(:,1) = 0*ones(lx,1);
yol(:,1) = C*xol(:,1) + D2*wnoise(:,1);
zol(:,1) = E1*xol(:,1) - r;

theta(:,1) = FLAG.theta_0;


%Create S-Function variables
rcactype = 'RLS';
P0 = FLAG.Rtheta^-1; P0 = 1e5*eye(ltheta);
mexFLAGS = [lz;ly;lu;Nc;Hnum+1;k0+1;P0(:);FLAG.Ru(:);FLAG.Rz(:);FLAG.lambda;FLAG.theta_0]';
mexNu = FILT.Nu; mexDu = FILT.Du; mexNz = FILT.Nz; mexDz = FILT.Dz;
mexFILT = [mexNu(:);mexDu(:);mexNz(:);mexDz(:)]';

%rcactype = 'Grad';
%[lz;ly;lu;Nc;filtorder;k0;alpha;theta_0]
%mexFLAGS = [lz;ly;lu;Nc;Hnum+1;k0+5;0.1;FLAG.theta_0]';

rcactype = 'Cumgrad';
%alpha = step scaling, gamm = regularization, lambda = forgetting
%[lz;ly;lu;Nc;filtorder;k0;alpha;gamma;lambda;theta_0]
mexFLAGS = [lz;ly;lu;Nc;Hnum+1;k0+5;0.1;0;1;FLAG.theta_0]';

Gtest = tf(ss(A,B,C,[0],ts));

sim('simulinkmdl')
