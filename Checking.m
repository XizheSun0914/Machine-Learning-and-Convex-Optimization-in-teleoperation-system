psi_11 = -K+Dm*eye(2)+0.25*Ds/lambda_s*exp(gamma*Ds)*P'*P+0.5*delta_m*eye(2)+0.5*gamma*Mm*eye(2);
psi_22 = -K+Ds*eye(2)+0.25*Dm/lambda_m*exp(gamma*Dm)*P'*P+0.5*delta_s*eye(2)+0.5*gamma*Ms*eye(2);
psi_33 = -2*B*eye(2)+Ds/(1-lambda_s)*exp(gamma*Ds)*B^2*eye(2)+Dm/(1-lambda_m)*exp(gamma*Dm)*B^2*eye(2)+0.5*gamma*k*eye(2);
psi_12 = zeros(2);
psi_21 = zeros(2);
psi_13 = 0.5*(-P+k*eye(2));
psi_31 = 0.5*(-P+k*eye(2));
psi_23 = 0.5*(P-k*eye(2));
psi_32 = 0.5*(P-k*eye(2));
Psi = [psi_11,psi_12,psi_13;psi_21,psi_22,psi_23;psi_31,psi_32,psi_33];
eig(Psi)