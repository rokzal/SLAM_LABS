%-------------------------------------------------------
function compatibility = compute_compatibility (prediction, observations)
%-------------------------------------------------------
global  chi2;

% Compute individual distances
compatibility.d2 = zeros(observations.m, prediction.n);

for i = 1:observations.m
     [~, ~, indi] = obs_rows(i);
     z = observations.z(indi);
     R = observations.R(indi,indi);
     for j = 1:prediction.n
         [~, ~, indj] = obs_rows(j);
         y = z - prediction.h(indj);
         S = prediction.HPH(indj,indj) + R;
         compatibility.d2(i,j) = mahalanobis(y, S);
     end
end

%dof = 2*observations.m;
dof = 2;

compatibility.ic = compatibility.d2 < chi2(dof);
compatibility.candidates.features = find(sum(compatibility.ic, 1));
compatibility.candidates.observations = find(sum(compatibility.ic, 2))';

compatibility.AL = (sum (compatibility.ic, 2))';
compatibility.HS = prod(compatibility.AL + 1);


%%Teselated map

compatibility.tess = zeros()

