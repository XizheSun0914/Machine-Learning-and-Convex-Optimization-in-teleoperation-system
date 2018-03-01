function U = Upsilon(threshold,gamma,Q,T)

if abs(threshold)>gamma*abs(Q)
    U = 0;
else
    U = T/Q;
end