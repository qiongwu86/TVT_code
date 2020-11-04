clear all;
clc;

%%  802.11p protocol simulation
load('D:\\Experiments\\Simulation\\data\\Ncs.mat');
load('D:\\Experiments\\Simulation\\data\\p0.mat');
load('D:\\Experiments\\Simulation\\data\\p1.mat');
load('D:\\Experiments\\Simulation\\data\\w0.mat');
load('D:\\Experiments\\Simulation\\data\\w1.mat');

N=7000;
% 802.11p parameters
daodalv_0=20;
daodalv_1=20; % lambda
slot_time=13*(10^(-6)); % slot time 

PHYh=48;MACh=112;Pl=1600;% bits
Rd=6*10^6;Rb=10^6;tran_time=10^(-6);
Ttr=(PHYh/Rb)+((MACh+Pl)/Rd)+2*tran_time;% average transmission time
Pa0=1-exp(-daodalv_0*slot_time); % packet arrival probability
Pa1=daodalv_1*slot_time; 

W0_0=4; 
W1_0=4;W1_1=8;

AIFSN0=2;
AIFSN1=3;
SIFS=32*(10^(-6));
AIFS0=SIFS+AIFSN0*slot_time;
AIFS1=SIFS+AIFSN1*slot_time;

M1=1;
L_limit=1;
L1=L_limit+M1; % retransmission limit
A0=0;A1=AIFSN1-AIFSN0;

%！！AC0 simulation-----%
TransmissionProbability_Sim0=zeros(1,N+1);
CollisionProbability_Sim=zeros(1,N+1);

 for tt=1:1:N+1;
Counter=0;           
TotalTimeInSeconds=1.3;% total simulation time (s)
Start = 0;      
ArrivalTime = 1:Ncs(1,tt);  % the instance when a new packet arrives 
PacketLength = 1:Ncs(1,tt);   % length of new packet
HasPacket = zeros(1,Ncs(1,tt));
CW = 1:Ncs(1,tt);
BackoffTimer = 1:Ncs(1,tt);
PacketBuff = zeros(Ncs(1,tt),5001); % Matrix of pacekt size in the waiting spacing for each node.
CollitionStations = zeros(1,Ncs(1,tt)+1);  % number of transmitting vehicle
SlotTime = 13*10^(-6);      % slot time
TotalTime = floor(TotalTimeInSeconds/SlotTime);  % total simulation time��(slot)
CurBufferSize = zeros(1,Ncs(1,tt)); 
ChannelBusy=zeros(1,TotalTime);
Transmissionnum=0;
Collisionnum=0;

%initialize
for i = 1:Ncs(1,tt)
    ArrivalTime(i) = 1;  % initial the arrival time of packet in each vehicle
    CW(i) = W0_0; % contention window
    BackoffTimer(i) = W0_0-1; % backoff counter
end

for t = 1:TotalTime %total simulation time (slot)
    for i = 1:Ncs(1,tt) % vehicle
     % initialize
        if t == ArrivalTime(i)
            if(rand<=p0(1,tt)) % has packt waiting 
                PacketLength(i) = 164;  
                PacketBuff = Push(PacketBuff,i,PacketLength(i));% push the new packet
                CurBufferSize(i) = CurBufferSize(i) + PacketLength(i);
                HasPacket(i) = 1; % has packet waiting 
                BackoffTimer(i) = RandSel(CW(i)); % backoff counter randomly choose a value and start a backoff stage            
            else                  % no packet waiting in the queue of AC0
                BackoffTimer(i) = RandSel(CW(i));
            end
            Counter=Counter+1;           
        end
    end

    for i = 1:Ncs(1,tt)
        if HasPacket(i) == 1  % there are packets waiting to transmit in the queue of ACO
            if BackoffTimer(i) == 0 % value of backoff counter is zero
                Transmissionnum=Transmissionnum+1;
                Counter=Counter+1;          
                CollitionStations = Add(CollitionStations,i); % number of transmitting vehicle increases by one           
                Start = 1; % channel becomes busy
            elseif BackoffTimer(i) == 1 % value of backoff counter is not equal to zero
                if(Start==0)  % channel is idle
                    BackoffTimer(i) = BackoffTimer(i) - 1; % value of backoff counter increases by one 
                    Counter=Counter+1;
                    ChannelBusy(1,t+1)=1;
                else
                    BackoffTimer(i) = BackoffTimer(i);
                    Counter=Counter+1;
                end
            else
                if(Start==0)
                    BackoffTimer(i) = BackoffTimer(i) - 1;
                    Counter=Counter+1;
                else
                    BackoffTimer(i) = BackoffTimer(i);
                    Counter=Counter+1;
                end
            end
        elseif HasPacket(i) == 0 % no packet needs to transmit
            if BackoffTimer(i) == 0
                    if (rand>Pa0)% no packet arrives
                        BackoffTimer(i) = 0;
                        Counter=Counter+1;
                    else % packet arrives
                        HasPacket(i)=1;
                        PacketLength(i) = 164;                 
                        PacketBuff = Push(PacketBuff,i,PacketLength(i));                  
                        CurBufferSize(i) = CurBufferSize(i) + PacketLength(i);                                                                      
                        BackoffTimer(i) = RandSel(W0_0);
                        Counter=Counter+1;
                    end
            else
                if(Start==0)
                    BackoffTimer(i) = BackoffTimer(i) - 1;
                    Counter=Counter+1;
                else
                    BackoffTimer(i) = BackoffTimer(i);
                    Counter=Counter+1;
                end
            end
        end
    end
    

    % the process afer transmitting for AC0
    if Start == 1  
        n_c = CollitionStations(1); % number of transmitting vehicles
        for i = 1:n_c
            j = CollitionStations(i+1); % vehicle j
            CurBufferSize(j) = CurBufferSize(j) - PacketLength(j);
            PacketBuff = Pop(PacketBuff,j); % number of packets is decreased by one
            CW(j) = W0_0;
            k = PacketBuff(j,1); % vehicle j number of remaining packets
            
            if k ==0 %  no packet waits
                ArrivalTime(j) = 1 + t; % next slot time
                HasPacket(j) = 0;
                BackoffTimer(j) = RandSel(CW(j)); % randomly choose a value 
                Counter=Counter+1;
            else     % there are still packets waiting to transmit
                BackoffTimer(j) = RandSel(CW(j));
                Counter=Counter+1;
            end
        end     
        Start=0;% channel becomes idle        
        CollitionStations = zeros(1,Ncs(1,tt)+1);
    end 
end
TransmissionProbability_Sim0(1,tt)=(Transmissionnum)/Counter;
 end
          
 
%！！！！simulation AC1！！！！！%
TransmissionProbability_Sim1=zeros(1,N+1);
CollisionProbability_Sim_1=zeros(1,N+1);
for tt=1:1:N+1;
Counter_1=0;           
TotalTimeInSeconds_1=1.3;% total simulation time (s)
Start_1 = 0;      
ArrivalTime_1 = 1:Ncs(1,tt);       % the instance when a new packet arrives 
PacketLength_1 = 1:Ncs(1,tt);     % length of the new packet
HasPacket_1 = zeros(1,Ncs(1,tt));
CW_1 = 1:Ncs(1,tt);
BackoffTimer_1 = 1:Ncs(1,tt);
cc = 1:Ncs(1,tt);
PacketBuff_1 = zeros(Ncs(1,tt),5001); % Matrix of pacekt size in the spacing for each vehicle.
CollitionStations_1 = zeros(1,Ncs(1,tt)+1);  % number of transmitting vehicle
SlotTime_1 = 13*10^(-6);      % slot time
TotalTime_1 = floor(TotalTimeInSeconds_1/SlotTime_1); % total simulation time��(slot)
CurBufferSize_1 = zeros(1,Ncs(1,tt));
ChannelBusy_1=zeros(1,TotalTime_1);
Transmissionnum_1=0;
Collisionnum_1=0;

% initialize
for i = 1:Ncs(1,tt)
    ArrivalTime_1(i) = 1;  % initial the arrival time of packets 
    CW_1(i) = W1_0; % minimum contention window
    BackoffTimer_1(i) = W1_0-1;  % backoff counter
end

for t = 1:TotalTime_1
    for i = 1:Ncs(1,tt)
        % initialize
        if t == ArrivalTime_1(i)
            cc(i)=0;       % number of retransmission
            if(rand<=p1(1,tt))  % has packt waiting
                PacketLength_1(i) = 164;
                PacketBuff_1 = Push(PacketBuff_1,i,PacketLength_1(i));% push the new packet
                CurBufferSize_1(i) = CurBufferSize_1(i) + PacketLength_1(i);
                HasPacket_1(i) = 1;  % existing packet
                BackoffTimer_1(i) = RandSel(CW_1(i)); % backoff counter randomly choose a value and start a backoff stage               
            else                     % no packet waiting in the queue of AC0
                BackoffTimer_1(i) = RandSel(CW_1(i));
            end
            Counter_1=Counter_1+1;           
        end
    end

    for i = 1:Ncs(1,tt)
        if HasPacket_1(i) == 1  % there are packets waiting to transmit in the queue for AC1
            if BackoffTimer_1(i) == 0 % value of backoff counter is zero
                if(rand>w0(1,tt))     % no internel collision happens 
                    Transmissionnum_1=Transmissionnum_1+1;
                    Counter_1=Counter_1+1;          
                    CollitionStations_1 = Add(CollitionStations_1,i); % number of transmitting vehicle increases by one              
                    Start_1 = 1; % channel becomes busy
                else             % internel collision happens
                    if (cc(i)<=1)   % number of retransmission is less than retry limit 
                    CW_1(i) = Increase(CW_1,i,W1_0);  % double contention window
                    BackoffTimer_1(i) = RandSel(CW_1(i)); % Randomly choose a value from new contention window
                    Counter_1=Counter_1+1; 
                    cc(i)=cc(i)+1;   % number of retransmission increases by one
                     continue;
                    else            % number of retransmission reaches retry limit 
                        cc(i)=0;  % reset the number of retransmission increases by one
                        CW_1(i)=W1_0; % reset the contention window
                        PacketBuff_1 = Pop(PacketBuff_1,i); % discarded pacekt
                        if (PacketBuff_1(i,1)==0)  % no packet waits
                            ArrivalTime_1(i) = 1 + t;
                            BackoffTimer_1(i) = RandSel(CW_1(i));
                            HasPacket_1(i) =0;
                            Counter_1=Counter_1+1; 
                            continue;
                        else       % there are still packets waiting to transmit
                            BackoffTimer_1(i) = RandSel(CW_1(i));
                            Counter_1=Counter_1+1; 
                            continue;
                        end
                        
                    end
                end
                
            elseif BackoffTimer_1(i) == 1   % value of backoff counter is not zero
                if(Start_1==0)   % channel is idle 
                    BackoffTimer_1(i) = BackoffTimer_1(i) - 1; % value of backoff counter decreases by one 
                    Counter_1=Counter_1+1;
                    ChannelBusy_1(1,t+1)=1;
                else
                    BackoffTimer_1(i) = BackoffTimer_1(i);
                    Counter_1=Counter_1+1;
                end
            else
                if(Start_1==0)
                    BackoffTimer_1(i) = BackoffTimer_1(i) - 1;
                    Counter_1=Counter_1+1;
                else
                    BackoffTimer_1(i) = BackoffTimer_1(i);
                    Counter_1=Counter_1+1;
                end
            end
        elseif HasPacket_1(i) == 0   % no packet waiting to transmit
            if BackoffTimer_1(i) == 0
                    if (rand>Pa1) % no packet arrives
                        BackoffTimer_1(i) = 0;
                        Counter_1=Counter_1+1;
                    else    % packet arrives
                        HasPacket_1(i)=1; 
                        PacketLength_1(i) = 164;                 
                        PacketBuff_1 = Push(PacketBuff_1,i,PacketLength_1(i));                  
                        CurBufferSize_1(i) = CurBufferSize_1(i) + PacketLength_1(i);                                                                      
                            BackoffTimer_1(i) = RandSel(W1_0);
                            Counter_1=Counter_1+1;
                    end
            else     % value of backoff counter is not zero
                if(Start_1==0)      % channel is idle
                    BackoffTimer_1(i) = BackoffTimer_1(i) - 1; % value of backoff counter decreases by one
                    Counter_1=Counter_1+1;
                else
                    BackoffTimer_1(i) = BackoffTimer_1(i); % channel is busy; value of backoff counter keeps the value.
                    Counter_1=Counter_1+1;
                end
            end
        end
    end

    if Start_1 == 1
        n_c_1 = CollitionStations_1(1);  % number of transmitting vehicles
        for i = 1:n_c_1
            j = CollitionStations_1(i+1); % vehicle j
            CurBufferSize_1(j) = CurBufferSize_1(j) - PacketLength_1(j);
            PacketBuff_1 = Pop(PacketBuff_1,j); % number of packets is decreased by one
            CW_1(j) = W1_0;
            k_1 = PacketBuff_1(j,1); % vehicle j number of remaining packets
            if k_1 ==0 %  no packet waits
                ArrivalTime_1(j) = 1 + t;
                HasPacket_1(j) = 0;
                BackoffTimer_1(j) = RandSel(CW_1(j)); % randomly choose a value from the maximum contention windows 
                Counter_1=Counter_1+1;
            else     % there are still packets waiting to transmit
                BackoffTimer_1(j) = RandSel(CW_1(j));
                Counter_1=Counter_1+1;
            end
        end
        Start_1=0;  % channle becomes idle
        CollitionStations_1 = zeros(1,Ncs(1,tt)+1);
    end 
end
TransmissionProbability_Sim1(1,tt)=(Transmissionnum_1)/Counter_1;
end

%% mean and variance
 tao_sim=zeros(1,N+1);
 PK_sim=zeros(1,N+1);
 pb0_sim=zeros(1,N+1);
 Ts0_sim=zeros(1,N+1);
 Ds0_sim=zeros(1,N+1);
 
 pb1_sim=zeros(1,N+1);
 Ts1_sim=zeros(1,N+1);
 Ds1_sim=zeros(1,N+1);

for tt=1:1:N+1;  
    syms z;
    TR=z^Ttr;  
tao_sim(1,tt)=TransmissionProbability_Sim0(1,tt)+TransmissionProbability_Sim1(1,tt);
PK_sim(1,tt)=(1-tao_sim(1,tt))^(Ncs(1,tt)-1);
pb0_sim(1,tt)=1-(PK_sim(1,tt)*(1-w1(1,tt)))^(A0+1);
pb1_sim(1,tt)=1-(PK_sim(1,tt)*(1-w0(1,tt)))^(A1+1);

%！！！AC0！！！！！%
H0_sim=(1-pb0_sim(1,tt))*(z^slot_time)+pb0_sim(1,tt)*(z^(Ttr+AIFS0));
B0_0_sim=0;
 %B0,0(Z)
  for i=0:W0_0-1;
      B0_0_sim=B0_0_sim+(1/W0_0)*(H0_sim)^i;     
  end
  
  PTs0_sim=TR*B0_0_sim;                      
  diff0_0=diff(PTs0_sim);    
  diff0_1=diff(PTs0_sim,z,2);
  
  Ts0_sim(1,tt)=subs(diff0_0,z,1);  % mean (s)
  Ds0_sim(1,tt)=subs(diff0_1,z,1)+Ts0_sim(1,tt)-(Ts0_sim(1,tt))^2; % variance (s)
  
 %！！！AC1！！！！%
  H1_sim=(1-pb1_sim(1,tt))*(z^slot_time)+pb1_sim(1,tt)*(z^(Ttr+AIFS1));
  guodu_jia10_sim=0;guodu_jia11_sim=0;
  
  for i = 0:W1_0-1
      guodu_jia10_sim=guodu_jia10_sim+(H1_sim)^i;
  end
  
  for i=0:W1_1-1
      guodu_jia11_sim=guodu_jia11_sim+(H1_sim)^i;
  end

%  B1,0(z)
    B1_0_sim=(1/W1_0)*guodu_jia10_sim;         
%   B1,1(z)
    B1_1_sim=(1/W1_1)*guodu_jia11_sim;         
%   B1,2(z)
    B1_2_sim=B1_1_sim;       
   
   PTs1_sim=(1-w0(1,tt))*TR*(B1_0_sim+w0(1,tt)*B1_0_sim*B1_1_sim+((w0(1,tt))^2)*B1_0_sim*B1_1_sim*B1_2_sim)+((w0(1,tt))^(L1+1))*B1_0_sim*B1_1_sim*B1_2_sim;
   diff1_0=diff(PTs1_sim);   
   diff1_1=diff(PTs1_sim,2);          
   
   Ts1_sim(1,tt)=subs(diff1_0,z,1); % mean (s)
   Ds1_sim(1,tt)=subs(diff1_1,z,1)+Ts1_sim(1,tt)-(Ts1_sim(1,tt))^2; % variance (s)  
end
