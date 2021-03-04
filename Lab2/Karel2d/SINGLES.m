%-------------------------------------------------------
function H = SINGLES (prediction, observations, compatibility)
%-------------------------------------------------------

global chi2;
global configuration;

H = zeros(1, observations.m);

% You have observations.m observations, and prediction.n
% predicted features.
%
% For every observation i, check whether it has only one neighbour,
% feature, and whether that feature j  has only that one neighbour
% observation i.  If so, H(i) = j.
%
% You will need to check the compatibility.ic matrix
% for this:
%
% compatibility.ic(i,j) = 1 if observation i is a neighbour of
% feature j.
for i = 1:observations.m
    compat_features = compatibility.ic(i,:);
    if (sum(compat_features) ==1)
        [~,j] = max(compat_features);
        compat_measurments = compatibility.ic(:,j);
        if (sum(compat_measurments) == 1)
            H(i) = j;
        end
    else 
    end
end
            
configuration.name = 'SINGLES';
