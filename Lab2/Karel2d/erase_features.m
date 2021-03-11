function [map] = erase_features(map, unreliable)
n_unreliable = size(unreliable,2);
map.n = map.n - n_unreliable;
for i = fliplr(unreliable) 
    i_1 = 4+(i-1)*2;
    i_2 = 4+(i-1)*2+1;
    map.x(i_1:i_2) = []; 
    map.P(i_1:i_2,:) = [];
    map.P(:,i_1:i_2) = [];
    map.ground_id(i) = [];
    map.hits(i) = [];
    map.first(i) = [];
end 

end
