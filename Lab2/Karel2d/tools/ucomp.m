%-------------------------------------------------------
function [tac, Pac] = ucomp(tab, Pab, tbc, Pbc);
%-------------------------------------------------------

tac = tcomp(tab, tbc);
J1 = jacobian1(tab, tbc);
J2 = jacobian2(tab, tbc);
Pac = J1 * Pab * J1' + J2 * Pbc * J2';
