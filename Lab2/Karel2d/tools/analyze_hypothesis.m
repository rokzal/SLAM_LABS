%-------------------------------------------------------
function analyze_hypothesis (prediction, observations, H)
%-------------------------------------------------------
global  chi2 ;

for i=1:length(H),
    if H(i)
        d2 = joint_mahalanobis2 (prediction, observations, H(1:i));
        dof = 2*length(find(H(1:i)));
        if dof > 0
            answer = d2 < chi2(dof);
            fprintf('i: %d, d^2 %f, chi^2 %f, ans %d\n', i, d2, chi2(dof), answer);
        end
    end
end
disp(' ');