
% Gyw = Gzu;

%G = ss([0.25,0;0,0.25],[1;1],[1,0],[0],1);
%G = ss([0.25,1;0,0.25],[1,0;0,1],[1,0;0,3],[0],1);
G = ss([0.25,1;0,0.25],[1,0,1,0,1,0;0,1,0,1,0,1],[1,0;0,1;1,0;0,1;1,0;0,1;1,0;0,1;1,0],[0],1);

[A,Bsys,Csys,Dsys] = ssdata(G); %Gzu

E1 = Csys;
C = E1;
B = Bsys;
D1 = B;
D2 = Dsys;
D0 = Dsys;

Gzu = ss(A,B,E1,0,1);
Gzw = ss(A,D1,E1,0,1);
Gyu = ss(A,B,C,0,1);
Gyw = ss(A,D1,C,D2,1);

ly = size(C,1);
lu = size(B,2);
lz = size(E1,1);
lx = size(A,1);

transG = tzero(G);
Gtrans = ones(lu,lz)*tf(poly(transG),[1],1);

%{
figure(22)
iopzplot(G,minreal(Gtrans))
ll = legend('$G$','${\rm trans} (G)$');
set(ll,'Interpreter','Latex','FontSize',12)
%}