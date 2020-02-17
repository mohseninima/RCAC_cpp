clear all;
close all;
clc;

p = bodeoptions;
p.FreqScale = 'linear';
p.FreqUnits = 'Hz';
p.PhaseWrapping = 'on';
% p.Xlim = [0 0.5];
p.Ylim = {[-10 10],[-180 180]};

if ~(exist('plotplant'))
    plotplant = 1;
end

%% %---- % Gf = 1/z %Gf = Gzu
Plant10_NMPChzNMPTz

ts = 1;
steps = 30;
tend = steps*ts;

t = 0:ts:tend;
% wnoise = 1*ones(lu,length(t));
%wnoise = 0*[sin(0.1*t);sin(0.1*t)];
% wnoise = 1*randn(lu,length(t));
wnoise = 0*([sin(0.1*t);sin(0.1*t)]);
wnoise = zeros(lu,1)*sin(0.1*t);

Nc = 22;
firflag = 0;
sparseflag = 0;
gcstab = 0;
k0 = Nc;
%
Ru = 0e-10;
Ruf = 0;

Rt = 1e-5;
Rtnum = Rt;
Rtden = Rt;

Rd = 0e1;
kfq = 0e-3;
satlv = 10;
thetasat = 1;



Gf = G;
% for m = 1:lz
%     for n = 1:lu
%             Gf(m,n) = tf(poly(zero(Gf(m,n))),poly(0*pole(Gf(m,n))),1);
%     end
% end


%% Plant
%1 - y = z; u = w
%2 - y neq z; u = w
%3 - y eq z; u neq w

% PlantBuilderNMPConfig3

ts = 1;
tend = steps*ts;
t = 0:ts:tend;

% figure(1010)
% bode(Gzw,'g',Gyw,'k',Gyu,'r',Gzu,'b.',pplant)
% h = legend('$G_{zw}$','$G_{yw}$','$G_{yu}$','$G_{zu}$');
% set(h,'Interpreter','latex')

[numsys,densys] = tfdata(Gzu);
numsys = numsys{1};
densys = densys{1};

% Get matrices
% [A,B,C,D] = ssdata(Gzu);

%
% E1 = C;
% D1 = B;

lx = size(A,1);
lu = size(B,2);
lz = size(C,1);
ly = size(C,1);

r = ones(ly,1);

Hnum = 3;
[Y,T] = impulse(Gzu,Hnum);
Yhat = Y*ts;

for mm = 1:size(Y,2)
    for nn = 1:size(Y,3)
        Gf(mm,nn) = tf(squeeze(Yhat(:,mm,nn))',[1;0*squeeze(Yhat(1:end-1,mm,nn))]',ts);
    end
end
tic
Run_RCACFB
toc
%%
%{
figure(3)
subplot(2,1,1)
plot(t,yol(1,:),t,y(1,:))
yy = ylabel('$y(1)$');
set(yy,'Interpreter','Latex','FontSize',12)

figure(2)
subplot(2,1,1)
plot(t,zol(1,:),t,z(1,:))
yy = ylabel('$z(1)$');
set(yy,'Interpreter','Latex','FontSize',12)
%{
subplot(2,1,2)
plot(t,zol(2,:),t,z(2,:))
yy = ylabel('$z(2)$');
%}
set(yy,'Interpreter','Latex','FontSize',12)
xx = xlabel('Time Step $k$');
set(xx,'Interpreter','Latex','FontSize',12)
ll = legend('$z_{ol}$','$z$');
set(ll,'Interpreter','Latex','FontSize',12)

Gc = getcontRCACMIMO(theta,lu,ly,Nc,FLAG,ts)
figure(10)
iopzmap(Gc)
%}