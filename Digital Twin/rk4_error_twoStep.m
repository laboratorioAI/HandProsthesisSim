function [yNext2h, errFirstStep] = rk4_error_twoStep(f, t, y, h)
% England two-step error estimate for classical RK4 (p.168 England 1972)
% Returns: y(t+2h)  *and*  error belonging to the FIRST of the two h-steps.

% ----- one big 2h step -----------------------------------------
k1 = f(t      , y);
k2 = f(t+h    , y + h*k1/2);
k3 = f(t+h    , y + h*k2/2);
k4 = f(t+2*h  , y + 2*h*k3);
yBig = y + (2*h)/6 * (k1 + 2*k2 + 2*k3 + k4);

% ----- two successive h steps ----------------------------------
yHalf = y;  tau = t;
for j = 1:2
    k1h = f(tau      , yHalf);
    k2h = f(tau+h/2  , yHalf + h*k1h/2);
    k3h = f(tau+h/2  , yHalf + h*k2h/2);
    k4h = f(tau+h    , yHalf + h*k3h);
    yHalf = yHalf + h/6 * (k1h + 2*k2h + 2*k3h + k4h);
    tau   = tau + h;
end

errFirstStep = norm(yHalf - yBig, 2) / 30;   % England factor
yNext2h      = yHalf;                         % accurate value
end
