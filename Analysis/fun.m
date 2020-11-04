%% PSFFA equation of AC0
function z=fun(t,x,flag,c_0,u_0)     
  daodalv_0=20;
  g=x^2+2*x*(c_0)^2+1;
  g1=-(x+1-sqrt(g))/(1-(c_0)^2);
  z=g1*u_0+daodalv_0; % PSFFA equation
end