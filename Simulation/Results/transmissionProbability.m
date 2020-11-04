%{  
  Compare the simulation result of transmission probability with that of
    analysis result which is calculated according to Eq. (31) [35].
%}
clear all;
% simulation results
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_1\\TransmissionProbability_Sim0_1');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_1\\TransmissionProbability_Sim1_1');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_2\\TransmissionProbability_Sim0_2');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_2\\TransmissionProbability_Sim1_2');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_3\\TransmissionProbability_Sim0_3');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_3\\TransmissionProbability_Sim1_3');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_4\\TransmissionProbability_Sim0_4');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_4\\TransmissionProbability_Sim1_4');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_5\\TransmissionProbability_Sim0_5');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_5\\TransmissionProbability_Sim1_5');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_6\\TransmissionProbability_Sim0_6');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_6\\TransmissionProbability_Sim1_6');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_7\\TransmissionProbability_Sim0_7');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_7\\TransmissionProbability_Sim1_7');

load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_8\\TransmissionProbability_Sim0_8');
load('D:\\TVT_Sim\\Experiments\\Simulation\\Results\\s_8\\TransmissionProbability_Sim1_8');

% analysis results
load('D:\\TVT_Sim\\Experiments\\Simulation\\data\\w0.mat');
load('D:\\TVT_Sim\\Experiments\\Simulation\\data\\w1.mat');

%% Average the simulation results and select several points to draw.
Q=7001; K=8;
Tr0_sim=zeros(1,Q);
Tr1_sim=zeros(1,Q);

for i=1:Q
    temp_0 = TransmissionProbability_Sim0_1(1,i)+TransmissionProbability_Sim0_2(1,i)+TransmissionProbability_Sim0_3(1,i)+TransmissionProbability_Sim0_4(1,i)+TransmissionProbability_Sim0_5(1,i)+TransmissionProbability_Sim0_6(1,i)+TransmissionProbability_Sim0_7(1,i)+TransmissionProbability_Sim0_8(1,i);
    Tr0_sim(1,i) = temp_0/K;
    
    temp_1 = TransmissionProbability_Sim1_1(1,i)+TransmissionProbability_Sim1_2(1,i)+TransmissionProbability_Sim1_3(1,i)+TransmissionProbability_Sim1_4(1,i)+TransmissionProbability_Sim1_5(1,i)+TransmissionProbability_Sim1_6(1,i)+TransmissionProbability_Sim1_7(1,i)+TransmissionProbability_Sim1_8(1,i);
    Tr1_sim(1,i) = temp_1/K;
end

S=24;SS=3;KK=0;
Tr0_sim_select=zeros(1,S);
Tr1_sim_select=zeros(1,S);
w0_target=zeros(1,S);
w1_target=zeros(1,S);
for i=1:SS*100:Q
    KK=KK+1;
    Tr0_sim_select(1,KK)=Tr0_sim(1,i);  
    Tr1_sim_select(1,KK)=Tr1_sim(1,i);
    
    w0_target(1,KK) = w0(1,i);
    w1_target(1,KK) = w1(1,i);
end

% ！！！！！！accuracy！！！！！ %
S=24;
Tr_0cha=zeros(1,S);
Tr_1cha=zeros(1,S);

for i=1:S
    Tr_0cha(1,i)=abs(Tr0_sim_select(1,i)-w0_target(1,i))/w0_target(1,i);
    Tr_1cha(1,i)=abs(Tr1_sim_select(1,i)-w1_target(1,i))/w1_target(1,i);   
end
[Tr0maxDevi,m3]=max(Tr_0cha); % max deviation of transmission probability for AC0
[Tr1maxDevi,m4]=max(Tr_1cha); % max deviation of transmission probability for AC0


%% draw
gap =0.01;N=7000;
figure(1);
y=0:SS:70;
plot(y,Tr0_sim_select,'^g','linewidth',1.5);
set(gcf,'unit','centimeters','position',[3 5 10 8.5]) 
set(gca,'Position',[.15 .15 .8 .75]);
hold on
t=0:gap:N/100;
plot(t,w0,'--g','linewidth',1.5);
ylim([1.5*10^(-4),4*10^(-4)])
xlabel('t (second)');
ylabel('Transmission Probability');
legend('AC0-simulation','AC0-analysis');



figure(2);
y=0:SS:70;
plot(y,Tr1_sim_select,'dm','linewidth',1.5);
set(gcf,'unit','centimeters','position',[3 5 10 8.5]) 
set(gca,'Position',[.15 .15 .8 .75]);
hold on
t=0:gap:N/100;
plot(t,w1,'--m','linewidth',1.5);
ylim([1.5*10^(-4),4*10^(-4)])
xlabel('t (second)');
ylabel('Transmission Probability');
legend('AC1-simulation','AC1-analysis');



