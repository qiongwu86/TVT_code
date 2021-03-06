clear all;
close all;

% calculate accuracy
load('D:\\Experiments\\Simulation\\data\\Ts0_tag_theory.mat');
load('D:\\Experiments\\Simulation\\data\\Ts1_tag_theory.mat');
load('D:\\Experiments\\Simulation\\data\\Ds0_tag_theory.mat');
load('D:\\Experiments\\Simulation\\data\\Ds1_tag_theory.mat');
load('D:\\Experiments\\Simulation\\data\\Dt_0_theory.mat');
load('D:\\Experiments\\Simulation\\data\\Dt_1_theory.mat');
load('D:\\Experiments\\Simulation\\data\\PDR0_theory.mat');
load('D:\\Experiments\\Simulation\\data\\PDR1_theory.mat');

% draw analysis results
load('D:\\Experiments\\Simulation\\draw\\Ds0_tag.mat');
load('D:\\Experiments\\Simulation\\draw\\Ds1_tag.mat');
load('D:\\Experiments\\Simulation\\draw\\Ts0_tag.mat');
load('D:\\Experiments\\Simulation\\draw\\Ts1_tag.mat');
load('D:\\Experiments\\Simulation\\draw\\Dt_0.mat');
load('D:\\Experiments\\Simulation\\draw\\Dt_1.mat');
load('D:\\Experiments\\Simulation\\draw\\PDR0.mat');
load('D:\\Experiments\\Simulation\\draw\\PDR1.mat');

% preliminary data of the simulation test
load('D:\\Experiments\\Simulation\\data\\Ncs.mat');
load('D:\\Experiments\\Simulation\\data\\position_x.mat'); 
load('D:\\Experiments\\Simulation\\data\\position_y.mat');

% simulation results of averaging the results obtained by the eight simulation tests
load('D:\\Experiments\\Simulation\\Results\\average results\\Ds0_sim.mat');
load('D:\\Experiments\\Simulation\\Results\\average results\\Ds1_sim.mat');
load('D:\\Experiments\\Simulation\\Results\\average results\\Ts0_sim.mat');
load('D:\\Experiments\\Simulation\\Results\\average results\\Ts1_sim.mat');
load('D:\\Experiments\\Simulation\\Results\\average results\\tao_sim.mat');

P=10;
Mv=8; 
gap=0.01;
N=7000;
R=500;

% 802.11p parameters
daodalv_0=20;
daodalv_1=20; % lambda
%% performance
x0=zeros(1,N+1); 
u_0=zeros(1,N+1);
c_0=zeros(1,N+1);
Yt_0=zeros(1,N+1);
Dt_0_sim=zeros(1,N+1);
x0(1,1)=0.00722730738593509;
xt_0=cell(N+1,1);
Xt_0=zeros(1,N+1);
Xt_0(1,1)=x0(1,1);

x1=zeros(1,N+1);
u_1=zeros(1,N+1);
c_1=zeros(1,N+1);
Yt_1=zeros(1,N+1);
Dt_1_sim=zeros(1,N+1);
x1(1,1)=0.00731440317965779;
xt_1=cell(N+1,1);
Xt_1=zeros(1,N+1);
Xt_1(1,1)=x1(1,1);
Ds0_select=zeros(1,N+1);

%--------delay--------------%
 for i=1:1:N+1
   % ！！！！AC0！！！！！！%  
     u_0(1,i)=1/Ts0_sim(1,i);  
     c_0(1,i)=u_0(1,i)*sqrt(Ds0_sim(1,i)); 
     
    % solve differential equation for AC0
    [~,xt_0{i,1}]=ode45('fun',[(i-1)*gap,i*gap],x0(1,i),[],c_0(1,i),u_0(1,i)); 
    xtt_0=xt_0{i,1};
    Xt_0(1,i+1)=xtt_0(length(xtt_0),1);
    
    % calculate the change rate of average number of packets
    Yt_0(1,i)=-(x0(1,i)+1-sqrt((x0(1,i))^2+2*(c_0(1,i))^2*x0(1,i)+1))/(1-(c_0(1,i))^2)*u_0(1,i)+daodalv_0;
     
   if i<N+1
    x0(1,i+1)=xt_0{i,1}(length(xtt_0),1);  % value of Xt_0(1,i+1)
   end
     % delay
    if(i>1)
         Dt_0_sim(1,i)=Dt_0_sim(1,i-1)+(Yt_0(1,i)/daodalv_0*gap)*(10^3);
    else
        Dt_0_sim(1,1)=(Xt_0(1,1)/daodalv_0)*(10^3); 
    end

  % ！！！！AC1！！！！！！%
    u_1(1,i)=1/Ts1_sim(1,i);  
    c_1(1,i)=u_1(1,i)*sqrt(Ds1_sim(1,i)); 
  
    % solve differential equation for AC1
    [~,xt_1{i,1}]=ode45('fun1',[(i-1)*gap,i*gap],x1(1,i),[],c_1(1,i),u_1(1,i));  % solve differential equation for AC1
    
    xtt_1=xt_1{i,1};
    Xt_1(1,i+1)=xtt_1(length(xtt_1),1);
    
    if i<N+1
     x1(1,i+1)=xt_1{i,1}(length(xtt_1),1);
    end
    % curve fitting to determine utilization function
    n=5;
    c1=c_1(1,i);
    luo=[0.001,0.02,0.003,0.04,0.005,0.06,0.007,0.08,0.009,0.1];
    x_luo=zeros(1,length(luo));
    for j=1:length(luo)
        x_luo(1,j)=luo(1,j)+(((luo(1,j))^2)*(c1^2)*exp((-2*(1-luo(1,j)))/(3*luo(1,j)*(c1^2))))/(2*(1-luo(1,j)));
    end
    p=polyfit(x_luo,luo,n);

    t=zeros(1,n);
    for j=1:n+1
      t(1,j)=p(j);
    end

    g1=t(1,1)*((x1(1,i))^(n))+t(1,2)*((x1(1,i))^(n-1))+t(1,3)*((x1(1,i))^(n-2))+t(1,4)*((x1(1,i))^(n-3))+t(1,5)*((x1(1,i))^(n-4))+t(1,6)*((x1(1,i))^(n-5));
    z=-g1*u_1(1,i)+daodalv_1;    
    Yt_1(1,i)=z; % calculate the change rate of average number of packets
    
    % delay
    if(i>1)
         Dt_1_sim(1,i)=Dt_1_sim(1,i-1)+(Yt_1(1,i)/daodalv_1*gap)*(10^3);
    else
        Dt_1_sim(1,1)=(Xt_1(1,1)/daodalv_1)*(10^3);
    end
 end

% ！！！！！！！！！packet delivery ratio！！！！！！！！！！！！%
% ！！！！！AC0！！！！！！%
fin_0=zeros(1,N+1);
fout_0=zeros(1,N+1);
G_0=zeros(1,N+1);

for i=1:N+1
    fin_0(1,i)=daodalv_0; % flow in disturbed vehicle (taeget vehicle) for AC0
    fout_0(1,i)=fin_0(1,i)-Yt_0(1,i); % flow out disturbed vehicle for AC0
end

%！！！！！AC1！！！！%
fin_1=zeros(1,N+1);
fout_1=zeros(1,N+1);
G_1=zeros(1,N+1);

for i=1:N+1
    fin_1(1,i)=daodalv_1;% flow in
    fout_1(1,i)=fin_1(1,i)-Yt_1(1,i); % flow out
end

 h_ir=zeros((P*Mv),N+1);
 h_ij=zeros((P*Mv),N+1);
 p_exposed=zeros((P*Mv),N+1);
 h_rj=zeros((P*Mv),(P*Mv),N+1);
 liancheng=ones((P*Mv),(P*Mv),N+1);
 p_hidden=zeros((P*Mv),N+1);
 p_c=zeros((P*Mv),N+1);

 PDR0s=zeros((P*Mv),N+1);
 PDR0x=zeros((P*Mv),N+1);
 PDR0_sim=zeros(1,N+1);
 
 PDR1s=zeros((P*Mv),N+1);
 PDR1x=zeros((P*Mv),N+1);
 PDR1_sim=zeros(1,N+1); 
 
 for k=1:1:N+1;
     % calculate the collision probability caused by the exposed terminal
     for r=1:1:(P*Mv);
         if(sqrt((position_x(9,k)-position_x(r,k))^2+(position_y(9,k)-position_y(r,k))^2)<=R)
             h_ir(r,k)=1;
         else
             h_ir(r,k)=0;
         end
          p_exposed(r,k)=h_ir(r,k)*(1-(1-tao_sim(1,k))^(Ncs(1,k)-1));
     end
     
     %{
     calculate the collision probability caused by the hidden terminal and 
     collision probability when the target vehicle Vi,j transmits to vehicle Vk,l.
     %}     
     for j=1:1:(P*Mv);
         if(sqrt((position_x(9,k)-position_x(j,k))^2+(position_y(9,k)-position_y(j,k))^2)<=R)
             h_ij(j,k)=1;
         else
             h_ij(j,k)=0;
         end
     end
     for r=1:1:(P*Mv);
         for j=1:1:(P*Mv);
             if(sqrt((position_x(r,k)-position_x(j,k))^2+(position_y(r,k)-position_y(j,k))^2)<=R)
                 h_rj(r,j,k)=1;
             else
                 h_rj(r,j,k)=0;
             end
         end
     end
     for r=1:1:(P*Mv);
         for j=1:1:(P*Mv);
             if(j~=1)
                 liancheng(r,j,k)=liancheng(r,j-1,k)*((1-tao_sim(1,k))^(51*(1-h_ij(j,k))*h_rj(r,j,k)));
             else
                 liancheng(r,1,k)=((1-tao_sim(1,k))^(51*(1-h_ij(1,k))*h_rj(r,1,k)));
             end
         end
         p_hidden(r,k)=h_ir(r,k)*(1-liancheng(r,(P*Mv),k));
         p_c(r,k)=1-(1-p_exposed(r,k))*(1-p_hidden(r,k));% collision probability
     end
     
     for r=1:1:(P*Mv);
         if(h_ir(r,k)==1) % within the transmisison range
             if(r~=1)
                 if(r==9) % disturbed vehicle (target vehicle)
                   PDR0s(r,k)=PDR0s(r-1,k);
                   PDR0x(r,k)=PDR0x(r-1,k);
                 
                   PDR1s(r,k)=PDR1s(r-1,k);
                   PDR1x(r,k)=PDR1x(r-1,k);                     
                 else
                   PDR0s(r,k)=PDR0s(r-1,k)+fout_0(1,k)*h_ir(r,k)*(1-p_c(r,k)); %the traffic successfully received by the vehicles within the transmission range of the target vehicle for the serving traffic of AC0. 
                   PDR0x(r,k)=PDR0x(r-1,k)+fin_0(1,k)*h_ir(r,k); % the traffic that should be received by the vehicles for the arriving traffic of AC0
                 
                   PDR1s(r,k)=PDR1s(r-1,k)+fout_1(1,k)*h_ir(r,k)*(1-p_c(r,k)); %the traffic successfully received by the vehicles for AC1
                   PDR1x(r,k)=PDR1x(r-1,k)+fin_1(1,k)*h_ir(r,k); %the traffic that should be received by the vehicles for the arriving traffic of AC1  
                 end
                     
             else
                 PDR0s(1,k)=fout_0(1,k)*h_ir(1,k)*(1-p_c(1,k));
                 PDR0x(1,k)=fin_0(1,k)*h_ir(1,k);
                 
                 PDR1s(1,k)=fout_1(1,k)*h_ir(1,k)*(1-p_c(1,k));
                 PDR1x(1,k)=fin_1(1,k)*h_ir(1,k);                 
             end
         else % out of the transmisison range 
             
             if(r~=1)
                PDR0x(r,k)=PDR0x(r-1,k); % the traffic successfully received by the vehicles keeps the same.
                PDR0s(r,k)=PDR0s(r-1,k);% the traffic that should be received by the vehicles keeps the same.
                
                PDR1x(r,k)=PDR1x(r-1,k);
                PDR1s(r,k)=PDR1s(r-1,k);                
             else
                 PDR0s(1,k)=0; % the traffic successfully received by the first vehicle for AC0.
                 PDR0x(1,k)=0; % the traffic that should be received by the first vehicle for AC0.
                 
                 PDR1s(1,k)=0;
                 PDR1x(1,k)=0;                 
             end             
             
         end
     end
     PDR0_sim(1,k)=PDR0s((P*Mv),k)/PDR0x((P*Mv),k); % pacekt delivery ratio for AC0
     PDR1_sim(1,k)=PDR1s((P*Mv),k)/PDR1x((P*Mv),k); % pacekt delivery ratio for AC1     
 end
%}

%-------take a point every 3 seconds for comparison-------%
Q=24;SS=3;
Ts01_sim_select=zeros(1,Q);
Ts11_sim_select=zeros(1,Q);

Ds01_sim_select=zeros(1,Q);
Ds11_sim_select=zeros(1,Q);

Dt_01_select=zeros(1,Q);
Dt_11_select=zeros(1,Q);

PDR01_select=zeros(1,Q);
PDR11_select=zeros(1,Q);

KK=0;
Ds0_sim_tag=zeros(1,N+1);
Ds1_sim_tag=zeros(1,N+1);
for i=1:N+1
    Ds0_sim_tag(1,i)=sqrt(Ds0_sim(1,i))*10^3;  % standard variance (ms)
    Ds1_sim_tag(1,i)=sqrt(Ds1_sim(1,i))*10^3;
end

for i=1:SS*100:N+1
    KK=KK+1;
    Ts01_sim_select(1,KK)=Ts0_sim(1,i)*1000;  % mean (ms)
    Ts11_sim_select(1,KK)=Ts1_sim(1,i)*1000;
    
    Ds01_sim_select(1,KK)=Ds0_sim_tag(1,i);
    Ds11_sim_select(1,KK)=Ds1_sim_tag(1,i);
    
    Dt_01_select(1,KK)=Dt_0_sim(1,i);
    Dt_11_select(1,KK)=Dt_1_sim(1,i);
    
    PDR01_select(1,KK)=PDR0_sim(1,i);
    PDR11_select(1,KK)=PDR1_sim(1,i);
end

% ！！！！！！accuracy！！！！！ %
Q=24;
Dt_0cha=zeros(1,Q);
PDR0_cha=zeros(1,Q);

Dt_1cha=zeros(1,Q);
PDR1_cha=zeros(1,Q);

Ds0_sim_ms=zeros(1,Q);
Ds1_sim_ms=zeros(1,Q);
for i=1:Q
    Dt_0cha(1,i)=abs(Dt_01_select(1,i)-Dt_0_theory(1,i))/Dt_0_theory(1,i);
    PDR0_cha(1,i)=abs(PDR01_select(1,i)-PDR0_theory(1,i))/PDR0_theory(1,i);
     
    Dt_1cha(1,i)=abs(Dt_11_select(1,i)-Dt_1_theory(1,i))/Dt_1_theory(1,i);
    PDR1_cha(1,i)=abs(PDR11_select(1,i)-PDR1_theory(1,i))/PDR1_theory(1,i);    
end
[Dt_0cha_max,m3]=max(Dt_0cha); % max deviation of packet delay for AC0
[PDR0_cha_max,m4]=max(PDR0_cha); % max deviation of packet delivery ratio for AC0

[Dt_1cha_max,m31]=max(Dt_1cha);
[PDR1_cha_max,m41]=max(PDR1_cha);

%% plot
figure(1);
y=0:SS:70;
plot(y,Ts01_sim_select,'^m','linewidth',1.5);
set(gcf,'unit','centimeters','position',[3 5 10 8.5]) 
set(gca,'Position',[.15 .15 .8 .75]);
hold on
plot(y,Ts11_sim_select,'dg','linewidth',1.5);
hold on
t=0:gap:N/100;
plot(t,Ts0_tag,'-m','linewidth',1.5);
hold on
plot(t,Ts1_tag,'--g','linewidth',1.5);
ylim([0.350,0.385])
xlabel('t (second)');
ylabel('Mean Service time(ms)'); 
legend('AC0-simulation','AC1-simulation','AC0-analysis','AC1-analysis');

figure(2);
y=0:SS:70;
plot(y,Ds01_sim_select,'^m','linewidth',1.5);
set(gcf,'unit','centimeters','position',[3 5 10 8.5])
set(gca,'Position',[.15 .15 .8 .75]);
hold on
plot(y,Ds11_sim_select,'dg','linewidth',1.5);
hold on
t=0:gap:N/100;
plot(t,Ds0_tag,'-m','linewidth',1.5);
hold on
plot(t,Ds1_tag,'--g','linewidth',1.5);
ylim([0.03,0.11])
xlabel('t (second)');
ylabel('Standard Variance(ms)'); 
legend('AC0-simulation','AC1-simulation','AC0-analysis','AC1-analysis');

figure(3);
y=0:SS:70;
plot(y,Dt_01_select,'^m','linewidth',1.5);
set(gcf,'unit','centimeters','position',[3 5 10 8.5])
set(gca,'Position',[.15 .15 .8 .75]);
hold on
plot(y,Dt_11_select,'dg','linewidth',1.5);
hold on
t=0:gap:N/100;
plot(t,Dt_0,'-m','linewidth',1.5);
hold on
plot(t,Dt_1,'--g','linewidth',1.5);
ylim([0.20,0.8])
xlabel('t (second)');
ylabel('Packet Delay (ms)'); 
legend('AC0-simulation','AC1-simulation','AC0-analysis','AC1-analysis');

figure(4);
y=0:SS:70;
plot(y, PDR01_select, '^m','linewidth',1.5);
set(gcf,'unit','centimeters','position',[3 5 10.6 8.6])
set(gca,'Position',[.15 .15 .8 .75]);
hold on
plot(y, PDR11_select, 'dg','linewidth',1.5);
hold on
plot(t, PDR0, '-m','linewidth',1.5);
hold on
plot(t,PDR1,'--g','linewidth',1.5);
ylim([0.2,0.8])
xlabel('t (second)');
ylabel('Packet Delivery Ratio');
legend('AC0-simulation','AC1-simulation','AC0-analysis','AC1-analysis');

