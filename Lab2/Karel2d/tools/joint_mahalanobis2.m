%-------------------------------------------------------
function [d2k, Hk, Sk, hk, zk, Rk] = joint_mahalanobis2 (prediction, observations, H)
%-------------------------------------------------------

[~, i, j] = find(H);

[~, ~, indi] = obs_rows(i);
[~, ~, indj] = obs_rows(j);

zk = observations.z(indi);
hk = prediction.h(indj);
Rk = observations.R(indi,indi);
Sk = prediction.HPH(indj,indj) + Rk;
Hk = prediction.H(indj,:);

d2k = mahalanobis (zk - hk, Sk);
