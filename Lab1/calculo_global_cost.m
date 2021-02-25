r   = 1;
m   = 2;
P   = 1000;

c1  = r(m-r)^2;
c2  = (m-r)*((r+1)^2+(m-r)-2*r*(m-2*r));
c3  = ((r+1)^2*(2*r-m)+r*(m-r)+r*(2*r-m)^2);

k1  = c1/(3*(m-r)^3);
k2  = (c1*(m^2+3*m-9*r)/(6*(m-r))+c2/2)/(m-r)^2;
k3  = (c1*(5*r^2-8*r*m)/(6*(m-r))+c2*(m-3*r)/2)/(m-r)^2;
k4  = (c1*(r*m*(r-m)+6*r^3)/(6*(m-r))+c2*(2*r^2-r*m)/2)/(m-r)^2;

k5  = c1*(P-r)/(3*(m-r)^2;
k6  = (c1*(3*m-7*r)/(6*(m-1))+c2/2)*(P-r)/(m-r)^2;
k7  = (c1*(6*r^2-6*m*r+m^2)/(6*(m-r))+c2*(m-2*r)/2)*(P-r)/(m-r)^2;
k8  = c3*(P-r);

k9  = -(2*r+1);
k10 = 1/2*(2*P*r+P-r);
k11 = 1/2*(2*P^2

C1map = k1*P^3 + k2*P^2 + k3*P + k4;