function Best = RJC (prediction, observations, compatibility)

global chi2;
global configuration;
Pfail = 0.7;
Pgood = 0.5;
b = 4;
Best = zeros(1, observations.m);
i = 1;
t = 2;
while i <= t 
	%randomize prediction-observations.
    permutation = randperm(observations.m);
	[~,inverse_perm] = sort(permutation);
    
    H = zeros(1, observations.m);
	H = JCBB_star(1,H,prediction, observations, compatibility,b,permutation);
    
	if pairings(H) > pairings(Best)
		Best = H(inverse_perm);
	end
	Pgood = max(Pgood,pairings(Best)/observations.m);
	t = log(Pfail)/log(1-Pgood.^b);
	i = i+1;
	
end

configuration.name = 'RJC';

end

function H = JCBB_star (i_index,H,prediction, observations, compatibility,b,permutation)
    
    if (i_index > size(permutation,2))
        return
    end
    i = permutation(i_index);
	if pairings(H) == b
		H = NN_R(H,i_index+1,prediction, observations,compatibility,permutation);
	else
		for j = 1:prediction.n
		   if compatibility.ic(i,j) && JointCompatibility(H,i,j,prediction,observations,compatibility,permutation)
			   H(i_index) = j;
               H = JCBB_star(i_index+1,H,prediction,observations,compatibility,b,permutation);
		   end
		end
	end
end

function res = pairings(H)
    tmp = H > 1;
    res = sum(tmp);
end

function H = NN_R(H,i_start,prediction, observations, compatibility,permutation)
%-------------------------------------------------------
global chi2;
global configuration;

for i_index = i_start:observations.m
    i = permutation(i_index);
    D2min = compatibility.d2 (i, 1);
    nearest = 1;
    for j = 2:prediction.n
        Dij2 = compatibility.d2 (i, j);
        if Dij2 < D2min
            nearest = j;
            D2min = Dij2;
        end
    end
    if D2min <= chi2(2) && JointCompatibility(H,i,nearest,prediction,observations,compatibility,permutation)
        H(i_index) = nearest;
    else
        H(i_index) = 0;
    end    
end
end



function res = JointCompatibility(H,i_new,j_new,prediction,observations,compatibility,permutation)
    global chi2;
    %calculate index
    
    z = [];
    all_indi = [];
    all_indj = [];
    res = 0;
    i_index = find(H);
    j = H(i_index);
    
    i = permutation(i_index);
    
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
    
    %Make sure it is positive definite, dont really know why it fails.
    [V,D] = eig(S);                    
    d= diag(D);           
    d(d <= 1e-7) = 1e-7; 
    D_c = diag(d);        
    S = real(V*D_c*V');   
    
    mah = mahalanobis(y, S);
    dof = max(size(z));
    res =  mah < chi2(dof);
  
end
