%-------------------------------------------------------
function map = add_new_features (map, observations, step, H,sensor_range,resolution)
%-------------------------------------------------------

prediction = predict_observations (map);
compatibility = compute_compatibility (prediction, observations,map,sensor_range,resolution,1);
new = find((H == 0) & (compatibility.AL == 0));

if nnz(new)
    disp(['NEW: ' sprintf('%2d   ', new)]);
    map = add_features(map, observations, step, new);
end

