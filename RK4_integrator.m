function [ output ] = RK4_integrator( ode_fun, input)
    p0 = input.p;
    u0 = input.u;
    Ts = input.ts;
    nSteps = input.nSteps;
%     h = Ts/nSteps;
%     output.value = rk4_step(ode_fun,p0,u0,h);
    
    np = length(p0);
    nu = length(u0);
    h = Ts/nSteps;
    STEP = 1e-100;
    
    compute_sensitivities = ~isfield(input,'sens') || input.sens;
    
    pEnd = p0;
    A = eye(np);
    B = zeros(np,nu);
    for i = 1:nSteps
        p0 = pEnd;
        pEnd = rk4_step(ode_fun,p0,u0,h);
        if compute_sensitivities
            sensX = zeros(np,np); sensU = zeros(np,nu);
            for j = 1:np
                % imaginary trick for states
                pTemp1 = p0; pTemp1(j) = pTemp1(j) + STEP*sqrt(-1);
                pTemp1 = rk4_step(ode_fun,pTemp1,u0,h);
                
                sensX(:,j) = imag(pTemp1)./STEP;
            end
            for j = 1:nu
                % imaginary trick for controls
                uTemp1 = u0; uTemp1(j) = uTemp1(j) + STEP*sqrt(-1);
                pTemp1 = rk4_step(ode_fun,p0,uTemp1,h);
                
                sensU(:,j) = imag(pTemp1)./STEP;
            end
            % propagate sensitivities
            A = sensX*A;
            B = sensX*B + sensU;
        end
    end
    output.value = pEnd;
    if compute_sensitivities
        output.sensX = A;
        output.sensU = B;
    end
end



function p_next = rk4_step(ode_fun,p,u,h)
    k1 = ode_fun(0,p,u);
    k2 = ode_fun(0,p+h/2.*k1,u);
    k3 = ode_fun(0,p+h/2.*k2,u);
    k4 = ode_fun(0,p+h.*k3,u);
    p_next = p + h/6.*(k1+2*k2+2*k3+k4);
end