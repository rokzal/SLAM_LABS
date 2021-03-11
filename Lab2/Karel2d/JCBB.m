function H = JCBB (prediction, observations, compatibility)

global chi2;
global configuration;
H = zeros(1, observations.m);

H = JCBB_rec (H,1,H,prediction, observations, compatibility)

end

function Best = JCBB_rec (Best,i,H,prediction, observations, compatibility)

if i > observations.m 
    if pairings(H) > pairings(Best)
        Best = H;
    end
else 
    for j = 1:prediction.n
       if compatibility.ic(i,j) && JointCompatibility(H,i,j,prediction,observations,compatibility)
           H(i) = j;
           Best = JCBB_rec(Best,i+1,H,prediction,observations,compatibility);
       end
    end
    if pairings(H) + observations.m - i  > pairings(Best)
        Best = JCBB_rec(Best,i+1,H,prediction,observations,compatibility);
    end
end
end

function res = pairings(H)
    tmp = H > 1;
    res = sum(tmp);
end





function res = JointCompatibility(H,i_new,j_new,prediction,observations,compatibility)
    global chi2;
    %calculate index
    
    z = [];
    all_indi = [];
    all_indj = [];
    res = 0;
%     for i = 1:observations.m
%         j = H(i);
%         if j > 0
%             [~, ~, indi] = obs_rows(i);
%             [~, ~, indj] = obs_rows(j);
%             all_indi = [all_indi;indi];
%             all_indj = [all_indj;indj];
%         end
%     end
    i = find(H);
    j = H(i);
    [~, ~, indi] = obs_rows(i);
    [~, ~, indj] = obs_rows(j);
    all_indi = [all_indi;indi];
    all_indj = [all_indj;indj];
    
    [~, ~, indi] = obs_rows(i_new);
    [~, ~, indj] = obs_rows(j_new);
    all_indi = [all_indi;indi];
    all_indj = [all_indj;indj];
    
    z = observations.z(all_indi);
    R = observations.R(all_indi,all_indi);
    y = z - prediction.h(all_indj);
    S = prediction.HPH(all_indj,all_indj) + R;
    mah = mahalanobis(y, S);
    dof = max(size(z));
    res =  mah < chi2(dof);
end

