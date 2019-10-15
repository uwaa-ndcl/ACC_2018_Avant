function [K,attempts] = lqr_adapt(nx,nu,x0,u0,t0)
% To get LQR working, try different step sizes in the complex step
% differentiation for finding the A and B matrices (eq. 13 in the paper).
%
% We use this technique because this problem has poor scaling. As a result,
% the LQR algorithm is often on the border of working/not working. Although
% this may seem shaky, the integration (ode45) works fine, which we believe
% validates using this method to find the gain matrix.

K = NaN;  % in case all attempts fail

% step sizes
H = [1e-9, 2e-9, 5e-9, 8e-9,...
     1e-10, 2e-10, 5e-10, 8e-10,...
     1e-11, 2e-11, 5e-11, 8e-11,...
     1e-12, 2e-12, 5e-12, 8e-12,...
     1e-13, 2e-13, 5e-13, 8e-13];
 
 % loop over step sizes
for i=1:length(H)
    h = H(i);
    const = params();
    
    % A: Jacobian of dynamics (G(x,u)) with respect to x, evaluated at x0
    A = zeros(nx,nx); 
    for j=1:nx
        delx = 1i*h*[zeros(j-1,1); 1; zeros(nx-j,1)];
        A(:,j) = imag(dx(t0,x0+delx,u0,const))/h;
    end

    % B: Jacobian of dynamics (G(x,u)) with respect to u, evaluated at u0
    B = zeros(nx,nu);
    for j=1:nu
        delu = 1i*h*[zeros(j-1,1); 1; zeros(nu-j,1)];
        u_delu = @(t,x) u0(t,x) + delu;
        B(:,j) = imag(dx(t0,x0,u_delu,const))/h;
    end
    B = B(:,1:11);  % rotor torque 4 is no longer a control

    % LQR (last equation on page 7 of paper)
    Q = diag([(1e3)*ones(3,1);
             (1e3)*ones(3,1);
             (1e4)*ones(4,1); 
             (1e7)*ones(4,1);
             (1e-12)*ones(4,1);
             1*ones(3,1);
             1*ones(3,1); 
             1*ones(4,1);
             1*ones(4,1);
             (1e-2)*ones(4,1)]);
    R = diag([ones(4,1); ones(4,1); (1e2)*ones(3,1)]);
    
    brk = 1; % break flag
    try
        [K,S,e] = lqr(A,B,Q,R);
    catch ME
        brk = 0;
    end
    if brk==1
        break;
    end

end

% if no solution is found, attempts is a NaN
if isnan(K)
    attempts = NaN;
else
    attempts = i;
end