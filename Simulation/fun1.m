%% PSFFA equation for AC1
function z=fun1(t,x,flag,c_1,u_1)
% curve fitting to determine utilization function and PSFFA equation for AC1
    n=5;
    daodalv_1=20;
    c1=c_1;
    luo=[0.001,0.02,0.003,0.04,0.005,0.06,0.007,0.08,0.009,0.1];
    x_luo=zeros(1,length(luo));
    for j=1:length(luo)
    
     x_luo(1,j)=luo(1,j)+(((luo(1,j))^2)*(c1^2)*exp((-2*(1-luo(1,j)))/(3*luo(1,j)*(c1^2))))/(2*(1-luo(1,j)));

    end
    p=polyfit(x_luo,luo,n); 

    t=zeros(1,n);
    for j=1:n+1
      t(1,j)=p(j); % coefficients
    end

g1=t(1,1)*((x)^(n))+t(1,2)*((x)^(n-1))+t(1,3)*((x)^(n-2))+t(1,4)*((x)^(n-3))+t(1,5)*((x)^(n-4))+t(1,6)*((x)^(n-5)); % utilization function
z=-g1*u_1+daodalv_1; % PSFFA equation for AC1
end