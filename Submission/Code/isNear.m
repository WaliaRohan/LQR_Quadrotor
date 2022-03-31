function check = isNear(X_sys, X_tar)

distance = norm([(X_sys(1) - X_tar(1)) (X_sys(2) - X_tar(2)) (X_sys(3) - X_tar(3))]);

if (distance <= 0.4)
    check = true;
else
    check = false;
end