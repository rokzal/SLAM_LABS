%-------------------------------------------------------
function Corr=correlation(Cov)
%-------------------------------------------------------

sigmas = sqrt(diag(Cov))' ;
Corr=diag(1./sigmas)*Cov*diag(1./sigmas) ;

