function dx = f(x,u,d,p)

DynamicParameters = p(1:27);
PropParams = p(28:39);

tau = PropModel(x,u,PropParams);
dx  = ShipModel(x,tau,d,DynamicParameters);

end

