r = 1;
m = 2;
P = 1000;

onemaparray = [];
manymaparray = [];
for p = 1:1:1000
    [C1map, C_many] = get_cost(r,m,P,p);
    onemaparray = [onemaparray, C1map];
    manymaparray = [manymaparray, C_many];
end

plot(1:1:1000, onemaparray);
hold on
plot(1:1:1000, manymaparray);
xlabel('Size of local Map.');
ylabel('Number of operations.');
title('Theoretical number of operations given local map size.');

function [C1map,C_total] = get_cost(r,m,P,p)
%r   = 1;
%m   = 2;
%P   = 1000;

c1  = r(m-r)^2;
c2  = (m-r)*((r+1)^2+(m-r)-2*r*(m-2*r));
c3  = ((r+1)^2*(2*r-m)+r*(m-r)+r*(2*r-m)^2);

k1  = c1/(3*(m-r)^3);
k2  = (c1*(m^2+3*m-9*r)/(6*(m-r))+c2/2)/(m-r)^2;
k3  = (c1*(5*r^2-8*r*m)/(6*(m-r))+c2*(m-3*r)/2)/(m-r)^2;
k4  = (c1*(r*m*(r-m)+6*r^3)/(6*(m-r))+c2*(2*r^2-r*m)/2)/(m-r)^2;

k5  = c1*(P-r)/(3*(m-r)^2);
k6  = (c1*(3*m-7*r)/(6*(m-1))+c2/2)*(P-r)/(m-r)^2;
k7  = (c1*(6*r^2-6*m*r+m^2)/(6*(m-r))+c2*(m-2*r)/2)*(P-r)/(m-r)^2;
k8  = c3*(P-r);

k9  = -(2*r+1);
k10 = 1/2*(2*P*r+P-r);
k11 = 1/2*(2*P^2*r-3*P*r^2+P^2-P*r+15*r^3+14*r^2+2*r);
k12 = 1/2*(2*P^2*r-10*P*r^2+3*P^2*r^2-12*P*r^3-P*r-9*r^4-12*r^3-3*r^2);
k13 = (-1/2)*(4*P^2*r^2-12*P*r^3+3*P^2*r^3-9*P*r^4+P^2*r-3*P*r^2);
k14 = -r;
k15 = 1/6*(P*r+17*r^2);
k16 = 1/6*(3*P^2*r-8*P*r^2-13*r^3);
k17 = 1/6*(2*P^3*r-9*P^2*r^2+13*P*r^3);
k18 = r;
k19 = 3*r^2+3*r;
k20 = 2*r^3+2*r^2;

C_total = k9*p^4/(p-2)^2+...
    k10*p^3/(p-r)^2+...
    k11*p^2/(p-2)^2+...
    k12*p/(p-r)^2+...
    k13/(p-r)^2+...
    k14*p^3/(p-r)+...
    k15*p^2/(p-r)+...
    k16*p/(p-r)+...
    (k17+k8)/(p-r)+...
    (k18+k5)*p^2+...
    (k19+k6)*p+...
    k20+k7;

C1map = k1*P^3 + k2*P^2 + k3*P + k4;
end