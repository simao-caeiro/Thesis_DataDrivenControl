function [F,G,Qb,Rb,H,Fd,Gd,Hd,A,B,C,Yb] = GetBatchXiMatrices(Ad,Bd,Cd,N,P,Q,R,ref)
% Returns matrices for batch computation of state sequence and cost 
% functional using incremental augmented representation

nxd = size(Bd,1);
nu = size(Bd,2);
ny = size(Cd,1);

A = [Ad , zeros(nxd,ny) ; Cd*Ad , eye(ny)];
B = [Bd ; Cd*Bd];
C = [zeros(ny,nxd),eye(ny)];
nx = size(B,1);

Fd = [];
Gd = [];
Hd = [];
F = [];
G = [];
H = [];
Qb = [];
Rb = [];
Yb = [];
for n = 0:N
    % state matrices
    F = [F ; A^n];
    Fd = [Fd ; Ad^n];
    Gi = [];
    Gdi = [];
    for m = 0:N-1
        ni = n-m-1;
        Gaux = A^ni*B;
        Gdaux = Ad^ni*Bd;
        if ni < 0
            Gaux = Gaux*0;
            Gdaux = Gdaux*0;
        end
        Gi = [Gi , Gaux];
        Gdi = [Gdi , Gdaux];
    end
    G = [G ; Gi];
    Gd = [Gd ; Gdi];
    Yb = [Yb;ref];
    H = blkdiag(H,C);
    Hd = blkdiag(Hd,Cd);
    
    % cost matrices
    if n < N
        Qb = blkdiag(Qb,Q);
        Rb = blkdiag(Rb,R);
    else
        Qb = blkdiag(Qb,P);
    end
end

