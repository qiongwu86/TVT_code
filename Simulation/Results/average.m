%% 
% We performanced eight simulation and averaged the results to obtain the simulation results.

clear all;
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_1\\Ds0_sim0_1');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_1\\Ts0_sim0_1');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_1\\Ds1_sim1_1');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_1\\Ts1_sim1_1');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_1\\tao_sim1');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_2\\Ds0_sim0_2');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_2\\Ts0_sim0_2');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_2\\Ds1_sim1_2');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_2\\Ts1_sim1_2');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_2\\tao_sim2');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_3\\Ds0_sim0_3');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_3\\Ts0_sim0_3');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_3\\Ds1_sim1_3');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_3\\Ts1_sim1_3');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_3\\tao_sim3');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_4\\Ds0_sim0_4');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_4\\Ts0_sim0_4');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_4\\Ds1_sim1_4');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_4\\Ts1_sim1_4');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_4\\tao_sim4');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_5\\Ds0_sim0_5');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_5\\Ts0_sim0_5');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_5\\Ds1_sim1_5');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_5\\Ts1_sim1_5');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_5\\tao_sim5');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_6\\Ds0_sim0_6');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_6\\Ts0_sim0_6');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_6\\Ds1_sim1_6');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_6\\Ts1_sim1_6');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_6\\tao_sim6');


load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_7\\Ds0_sim0_7');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_7\\Ts0_sim0_7');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_7\\Ds1_sim1_7');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_7\\Ts1_sim1_7');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_7\\tao_sim7');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_8\\Ds0_sim0_8');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_8\\Ts0_sim0_8');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_8\\Ds1_sim1_8');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_8\\Ts1_sim1_8');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_8\\tao_sim8');

Q=7001;
Ds0_sim=zeros(1,Q);
Ts0_sim=zeros(1,Q);
Ds1_sim=zeros(1,Q);
Ts1_sim=zeros(1,Q);
tao_sim=zeros(1,Q);

K=8;
for i=1:Q
    
    guo_sum_Ds0=(Ds0_sim0_1(1,i)+Ds0_sim0_2(1,i)+Ds0_sim0_3(1,i)+Ds0_sim0_4(1,i)+Ds0_sim0_5(1,i)+Ds0_sim0_6(1,i)+Ds0_sim0_7(1,i)+Ds0_sim0_8(1,i))/K;
    Ds0_sim(1,i)=guo_sum_Ds0;
    
    guo_sum_Ts0=(Ts0_sim0_1(1,i)+Ts0_sim0_2(1,i)+Ts0_sim0_3(1,i)+Ts0_sim0_4(1,i)+Ts0_sim0_5(1,i)+Ts0_sim0_6(1,i)+Ts0_sim0_7(1,i)+Ts0_sim0_8(1,i))/K;
    Ts0_sim(1,i)=guo_sum_Ts0;    
    
    guo_sum_Ds1=(Ds1_sim1_1(1,i)+Ds1_sim1_2(1,i)+Ds1_sim1_3(1,i)+Ds1_sim1_4(1,i)+Ds1_sim1_5(1,i)+Ds1_sim1_6(1,i)+Ds1_sim1_7(1,i)+Ds1_sim1_8(1,i))/K;
    Ds1_sim(1,i)=guo_sum_Ds1;
    
    guo_sum_Ts1=(Ts1_sim1_1(1,i)+Ts1_sim1_2(1,i)+Ts1_sim1_3(1,i)+Ts1_sim1_4(1,i)+Ts1_sim1_5(1,i)+Ts1_sim1_6(1,i)+Ts1_sim1_7(1,i)+Ts1_sim1_8(1,i))/K;
    Ts1_sim(1,i)=guo_sum_Ts1;   
    
    guo_sum_tao=(tao_sim1(1,i)+tao_sim2(1,i)+tao_sim3(1,i)+tao_sim4(1,i)+tao_sim5(1,i)+tao_sim6(1,i)+tao_sim7(1,i)+tao_sim8(1,i))/K;
    tao_sim(1,i)=guo_sum_tao;    
    
end  

save('Ts0_sim.mat','Ts0_sim');    
save('Ts1_sim.mat','Ts1_sim');

save('Ds0_sim.mat','Ds0_sim');
save('Ds1_sim.mat','Ds1_sim');

save('tao_sim.mat','tao_sim');
%}