km1 = dm*Qm+0.5*sigma*Mm+0.25*ds/(1-eta)*Es*P/Qs*P+0.5*lambda;
km2 = dm*Qm+0.5*sigma*Mm+0.25*ds*Es*P/Qs*P+0.5*lambda;
Km = max(km1,km2);
ks1 = ds*Qs+0.5*sigma*Ms+0.25*dm/(1-eta)*Em*P/Qm*P+0.5*lambda;
ks2 = ds*Qs+0.5*sigma*Ms+0.25*dm*Em*P/Qm*P+0.5*lambda;
Ks = max(ks1,ks2);
