%-------------------------------------------------------
function result = select_observations (observations, which)
%-------------------------------------------------------

[~, ~, ind] = obs_rows(which);
result.z = observations.z(ind);
result.R = observations.R(ind,ind);
result.m = length(which);
result.ground_id = observations.ground_id(which);
