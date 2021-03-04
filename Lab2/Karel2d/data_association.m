%-------------------------------------------------------
function [H, GT, compatibility] = data_association(map, observations, step),
%-------------------------------------------------------
global configuration ground;

% individual compatibility
prediction = predict_observations (map);
compatibility = compute_compatibility (prediction, observations);

% ground truth
GT = ground_solution(map, observations);
disp(['GROUND  TRUTH: ' sprintf('%2d  ', GT)]);

% your algorithms here!

% 1. Try NN. Complete SINGLES and try it. Include people and try SINGLES. Try NN and SINGLES without odometry.
% 2. Improve the computational cost of NN and SINGLES any way you can. Tip: reducing the big  O() is the idea worth more of your time, although other improvements are possible.
% 3. Implement map maintenance: eliminate all features seen only once (why this?), more than two steps ago. Is this always a good idea?
% 4. Have some time in your hands? Program JCBB and try it.∗
% 5. Still some time in your hands? Randomize Joint Compatibility, combining ideas from RANSAC and JCBB.∗∗

H = NN (prediction, observations, compatibility);

disp(['MY HYPOTHESIS: ' sprintf('%2d  ', H)]);
disp(['Correct (1/0)? ' sprintf('%2d  ', GT == H)]);
disp(' ');

draw_map (map, ground, step);
draw_observations (observations, ground, step);

draw_compatibility (prediction, observations, compatibility);

draw_hypothesis (prediction, observations, compatibility, H, configuration.name);
draw_tables (compatibility, GT, H);
