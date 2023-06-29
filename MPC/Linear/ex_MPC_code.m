%% Exercise book solutions (CPCS)
%  Module 2: Linear model prdictive control
%
% Bruno Guerreiro (bj.guerreiro@fct.unl.pt)

clear all;
clc
close all

%% MPC

param = load_crazyflie_parameters();

Ad = param.Ad_MPC;
Bd = param.Bd_MPC;
Cd = eye(9);

Q = diag([40, 40, 500, 0, 0, 0, 0, 0, 40]);
R = diag([0.5, 20, 20, 20]);
P = Q;
N = 50;
xd0 = zeros(9,1);
nxd = size(Bd,1);
nu = size(Bd,2);
ny = size(Cd,1);
nx = nxd+ny;

ref = [1; 1; -1; 0; 0; 0; 0; 0; 0];

% compute batch matrices
[F,G,Qb,Rb,H,Fd,Gd,Hd,~,~,~,Yb] = GetBatchXiMatrices(Ad,Bd,Cd,N,P,Q,R,ref);
Fb = H*F;
Gb = H*G;
Fdb = Hd*Fd;
Gdb = Hd*Gd;
Rt = Gb'*Qb*Gb + Rb;
St = Gb'*Qb;
Ky = Rt^(-1)*St;
K = Rt^(-1)*St*Fb;


%simulate controlled system:
nk = 250;

x0 = [xd0*0 ; Cd*xd0];
U = zeros(nu,nk);
U2 = zeros(nu,nk);
Xd(:,1) = xd0;
X(:,1) = x0;
Xd(:,2) = xd0;
X(:,2) = x0;

for k = 2:nk    
    Dxdk = Xd(:,k)-Xd(:,k-1);
    xk = [ Dxdk; Cd*Xd(:,k)];
    
    % compute unconstrained optimal sequence and MPC policy
    dUopt(:,:,k) = reshape(-(K*xk-Ky*Yb) ,nu,N);
    Uopt(:,:,k) = U(:,k-1) + dUopt(:,:,k);
    U(:,k) = Uopt(:,1,k);
    
    % simulate system (for comparison):
    Xd(:,k+1) = Ad*Xd(:,k) + Bd*U(:,k);
    
end

time = 0:param.sample_time_controller_outer:nk*param.sample_time_controller_outer;

figure()
subplot(3,1,1)
plot(time,Xd(1,:))
ylabel('Position X[m]')

subplot(3,1,2)
plot(time,Xd(2,:))
ylabel('Position Y[m]')

subplot(3,1,3)
plot(time,Xd(3,:))
ylabel('Position Z[m]')
xlabel('Time [s]')

figure()
subplot(3,1,1)
plot(time,Xd(7,:))
ylabel('\phi [\circ]')

subplot(3,1,2)
plot(time,Xd(8,:))
ylabel('\theta [\circ]')

subplot(3,1,3)
plot(time,Xd(9,:))
ylabel('\psi [\circ]')
xlabel('Time [s]')


figure()
plot(time(1:end-1),U(1,:))
