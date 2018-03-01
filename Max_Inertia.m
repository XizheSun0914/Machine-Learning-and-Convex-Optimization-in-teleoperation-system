function Mmax = Max_Inertia(m1, m2, l1, l2)
Mmax = 0;
    for q2=-pi/2:pi/200:pi/2
        M11 = m2*l2^2+(m1+m2)*l1^2+2*m2*l1*l2*cos(q2);
        M12 = m2*l2^2+m2*l1*l2*cos(q2);
        M21 = M12;
        M22 = m2*l2^2;
        M = [M11, M12; M21, M22];
        if max(eig(M-Mmax*eye(2)))>0
            Mmax = max(eig(M));
        end
    end