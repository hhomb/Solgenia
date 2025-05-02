function x_next=RK4_step(x,u,p,d,h)
    k1 = f(x,u,d,p);
    k2 = f(x+h/2*k1,u,d,p);
    k3 = f(x+h/2*k2,u,d,p);
    k4 = f(x+h*k3,u,d,p);
    x_next = x+h/6*(k1+2*k2+2*k3+k4);
end

