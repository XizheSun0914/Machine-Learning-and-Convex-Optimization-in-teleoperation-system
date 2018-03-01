function S = Gains(Dm, Ds, lambda_m, lambda_s, delta_m, delta_s, Mm, Ms, gamma, k)
%% find proper control gains
S=[];
Em=exp(gamma*Dm);
Es=exp(gamma*Ds);
for Pm=1:1:50
    Ps=Pm;
    K1=Dm+0.25*Ds*(1./lambda_s)*Es*Pm^2+0.5*delta_m+0.5*gamma*Mm;
    K2=Ds+0.25*Dm*(1./lambda_m)*Em*Ps^2+0.5*delta_s+0.5*gamma*Ms;
    K=ceil(max(K1,K2));
    for Km=K:1:100
        Ks=Km;
        for Bm=0:0.1:1
            Bs=Bm;
            A11=-Km+K1;
            A22=-Ks+K2;
            A33=-Bm-Bs+Ds*(1./(1-lambda_s))*Es*Bm^2+Dm*(1./(1-lambda_m))*Em*Bs^2+0.5*gamma*k;
            A13=0.5*(k-Pm);
            A31=A13;
            A23=0.5*(Ps-k);
            A32=A23;
            C13=A11*A33-A13*A31;
            C23=A22*A33-A23*A32;
            C123=-A11*A22*A33+A22*A13*A31+A11*A23*A32;
            if A11<=0 && A22<=0 && A33<=0 && C13>=0 && C23 >=0 && C123>=0
                S=[S;Km, Ks, Pm, Ps, Bm, Bs];
            end
        end
    end
end

