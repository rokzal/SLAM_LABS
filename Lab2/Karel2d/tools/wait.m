%-------------------------------------------------------
function wait(message)
%-------------------------------------------------------

if nargin == 0
    message = 'Press RETURN to continue...';
end

disp(message);
pause;