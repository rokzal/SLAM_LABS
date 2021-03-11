%-------------------------------------------------------
function compatibility = compute_compatibility (prediction, observations,map,sensor_range,resolution,full)
%-------------------------------------------------------
global  chi2;
%Compute features within sensor range 
% 
% robot_pos = [map.x(1);map.x(2)];
% robot_pos = robot_pos - map.origin';
% map_low = ceil((robot_pos-sensor_range+0.2) /resolution);
% map_high = ceil((robot_pos+sensor_range+0.2) /resolution);
% x_low = max(1,map_low(2));
% x_high = min(size(map.teselated,2),map_high(1));
% y_low = max(1,map_low(1));
% y_high = min(size(map.teselated,1),map_high(2));
% 
% sub_map = map.teselated(y_low:y_high,x_low:x_high); 
% index = find(sub_map);
% features = sub_map(index);
% features = features';
% if max(size(features)) <= 1 
%     features = 1:prediction.n;
% end
% Compute individual distances
compatibility.d2 = zeros(observations.m, prediction.n)+10;

x = prediction.h(1:2:end);
y = prediction.h(2:2:end);
dist = sqrt((x-0).^2+(y-0).^2);
in_range = find(dist < (sensor_range+0.5) & x >= -0.5);
in_range = in_range';
if full
    features = 1:prediction.n;
else 
    features = sort(in_range);
end
for i = 1:observations.m
     [~, ~, indi] = obs_rows(i);
     z = observations.z(indi);
     R = observations.R(indi,indi);
     %for j = 1:prediction.n
     for j = features
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

compatibility.tess = zeros();

