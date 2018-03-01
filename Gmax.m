function Gm = Gmax(Prm)

g = 9.8;
gm1 = 0;
gm2 = 0;
m1 = Prm(1);
m2 = Prm(2);
l1 = Prm(3);
l2 = Prm(4);

for q1 = -pi:0.1:pi
    for q2 = -pi:0.1:pi
        g1 = m2*g*l2*sin(q1+q2)+(m1+m2)*g*l1*sin(q1);
        g2 = m2*g*l2*sin(q1+q2);
        if gm1<abs(g1)
            gm1 = abs(g1);
        end
        if gm2<abs(g2)
            gm2 = abs(g2);
        end
    end
end
Gm = [gm1; gm2];