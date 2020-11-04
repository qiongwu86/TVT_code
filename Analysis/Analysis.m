%% connectivity model
clear all;
close all;

Vstb=25;
d_a=-2;% deceleration
P=10;% number of platoon
Mv=8;% number of vehicle
a=1.4;b=2;s0=3;v0=30;T0=1.5;T1=2;% IDM parameter
Stb=(s0+Vstb*T0)/(sqrt(1-(Vstb/v0)^4));
L=3;R=500;S=Stb+L;
DStb=(s0+Vstb*T1)/(sqrt(1-(Vstb/v0)^4)); % Distance at equilibrium
DS=DStb;


V=zeros(P,Mv,1);
X=zeros(P,Mv,1);
Y=zeros(P,Mv,1);
vcha=zeros(P,Mv,1);
scha=zeros(P,Mv,1);
sdes=zeros(P,Mv,1);
A=zeros(P,Mv,1);

sum=cell(3,1);
H=cell(1,1);

gap=0.01; % time interval
dec_t=1000;% duration of deceleration
stb_t=1000;% duration of vlow
N=7000;
width=3.5;

Ncs=zeros(1,N+1);
position_x=zeros((P*Mv),N+1);
position_y=zeros((P*Mv),N+1);
%************************************************************************
%initial velocity and position
%************************************************************************
flag_c=0;
for i=1:1:P;   % platoon
    for j=1:1:Mv;
        falg_c=flag_c+1;
        %lane 3 4 platoon
        if(i==1)   %  platoon 1
            Y(i,j)=1.5+width*1;
            V(i,j)=Vstb;   
            if(j==1)
                X(i,j)=2500;
                vcha(i,j)=0;
                scha(i,j)=0;
                A(i,j)=0;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;%inter-spacing
                A(i,j)=0;
            end
        end
        if(i==2)   % platoon 2
            Y(i,j)=1.5+width*1;
            V(i,j)=Vstb;   
            if(j==1)
                X(i,j)=X(i-1,Mv)-L-DS;
                vcha(i,j)=0;
                scha(i,j)=X(i-1,Mv)-X(i,j)-L;
                A(i,j)=d_a;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;
                A(i,j)=0;
            end
        end
        if(i==3)   % platoon 3
            Y(i,j)=1.5+width*1;
            V(i,j)=Vstb;   
            if(j==1)
                X(i,j)=X(i-1,Mv)-L-DS;
                vcha(i,j)=0;
                scha(i,j)=X(i-1,Mv)-X(i,j)-L;
                A(i,j)=0;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;
                A(i,j)=0;
            end
        end
        
        if(i==4)   % platoon 4
            Y(i,j)=1.5+width*1;
            V(i,j)=Vstb;  
            if(j==1)
                X(i,j)=X(i-1,Mv)-L-DS;
                vcha(i,j)=0;
                scha(i,j)=X(i-1,Mv)-X(i,j)-L;
                A(i,j)=0;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;
                A(i,j)=0;
            end
        end        
              
        %lane 4  
        if(i==5)   %  platoon 5
            Y(i,j)=1.5;
            V(i,j)=Vstb;   
            if(j==1)
                X(i,j)=1489.74602636741;
                vcha(i,j)=0;
                scha(i,j)=X(i-1,Mv)-X(i,j)-L;
                A(i,j)=0;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;
                A(i,j)=0;
            end
        end
        if(i==6)   %  platoon 6
            Y(i,j)=1.5;
            V(i,j)=Vstb;   
            if(j==1)
                X(i,j)=X(i-1,Mv)-DS-L;
                vcha(i,j)=0;
                scha(i,j)=X(i-1,Mv)-X(i,j)-L;
                A(i,j)=0;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;
                A(i,j)=0;
            end
        end
       
        %lane 2  
        if(i==7)   %  platoon 8
            Y(i,j)=1.5+width*2;
            V(i,j)=Vstb;   
            if(j==1)
                X(i,j)=1490;
                vcha(i,j)=0;
                scha(i,j)=X(i-1,Mv)-X(i,j)-L;
                A(i,j)=0;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;
                A(i,j)=0;
            end
        end
         if(i==8)   % platoon 9
            Y(i,j)=1.5+width*2;
            V(i,j)=Vstb;  
            if(j==1)
                X(i,j)=X(i-1,Mv)-DS-L;
                vcha(i,j)=0;
                scha(i,j)=X(i-1,Mv)-X(i,j)-L;
                A(i,j)=0;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;
                A(i,j)=0;
            end
         end       
         
        %lane 1
         if(i==9)   %  platoon 9
            Y(i,j)=1.5+width*3;
            V(i,j)=Vstb;   
            if(j==1)
                X(i,j)=1510;
                vcha(i,j)=0;
                scha(i,j)=X(i-1,Mv)-X(i,j)-L;
                A(i,j)=0;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;
                A(i,j)=0;
            end
         end
         if(i==10)   % platoon 10 
            Y(i,j)=1.5+width*3;
            V(i,j)=Vstb;   
            if(j==1)
                X(i,j)=X(i-1,Mv)-DS-L;
                vcha(i,j)=0;
                scha(i,j)=X(i-1,Mv)-X(i,j)-L;
                A(i,j)=0;
            else
                X(i,j)=X(i,j-1)-S;
                vcha(i,j)=0;
                scha(i,j)=Stb;
                A(i,j)=0;
            end
         end
         position_x(falg_c,1)=X(i,j);
    end
end

sum{1,1}=V;
sum{2,1}=X;
sum{3,1}=scha;

% time-varying kinestate
for t=2:N+1
    %% disturbed vehicle decelerates
  if(t<=dec_t+1)
     for i=1:P    % platoon
        for j=1:Mv     % Mv number of vehicle 
          %lane 3   
          if (i<=4)
            % The kinestate of the leader vehicle of the platoon which is in the same lane with the target vehicle and is in the front of the target vehicle
            if(j==1)&&(i==1)
                 V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                 X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                 vcha(i,j,t)=0;
                 scha(i,j,t)=0;
                 sdes(i,j,t)=0;
                 A(i,j,t)=0;
            % The kinestate of leader vehicle for other platoon in the same lane
            else if(j==1)&&(i~=1)&&(i~=2)
                % the time-varying kinestate of the leader vehicle for other platoons, except for the target vehicle.  
               if((X(i-1,Mv,t-1)-X(i,j,t-1))<=DS)  % sense the distance: less than distance at equilibrium
                vcha(i,j,t-1)=(V(i,j,t-1)-V(i-1,Mv,t-1));
                scha(i,j,t-1)=(X(i-1,Mv,t-1)-X(i,j,t-1)-L); % intra-platoon spacing
                sdes(i,j,t-1)=s0+V(i,j,t-1)*T1+(V(i,j,t-1)*vcha(i,j,t-1))/(2*sqrt(a*b)); % desired intra-platoon spacing
                A(i,j,t-1)=a*(1-(V(i,j,t-1)/v0)^4-(sdes(i,j,t-1)/scha(i,j,t-1))^2); % IDM model
                
                V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
               else if((X(i-1,Mv,t-1)-X(i,j,t-1))>DS)
                 if(V(i,j,t-1)>=Vstb)
                     A(i,j,t-1)=0;
                     V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                     X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                     vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                     scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                     sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                 else
                     A(i,j,t-1)=a;
                     V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                     X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                     vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                     scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                     sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                 end
                   end
               end
            % the time-varying kinestate of the disturbed vehicle
            else if(j==1)&&(i==2)  
            V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
            X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
            vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
            scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
            sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
            A(i,j,t)=A(i,j,t-1); % continue to decelarating and keeps the deceleration
                end
                end
            end
            % the time-varying kinestate of member vehicle
           if(j~=1)
              V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
              X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
              vcha(i,j,t)=(V(i,j,t-1)-V(i,j-1,t-1))+(A(i,j,t-1)-A(i,j-1,t-1))*gap;
              scha(i,j,t)=scha(i,j,t-1)+(V(i,j-1,t-1)-V(i,j,t-1))*gap+1/2*(A(i,j-1,t-1)-A(i,j,t-1))*gap^2;  % intra-platoon spacing
              sdes(i,j,t)=s0+V(i,j,t)*T0+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b)); % desired intra-platoon spacing
              A(i,j,t)=a*(1-(V(i,j,t)/v0)^4-(sdes(i,j,t)/scha(i,j,t))^2);
           end

        %lane 1,2,4  Since the deceleration of the disturbed vehicle does not affect the velocity of vehicles in other lanes, the acceleration of vehicle in other lanes is zero. 
        else
            V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
            X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
            vcha(i,j,t)=0;
            scha(i,j,t)=0;
            sdes(i,j,t)=0;
            A(i,j,t)=0;         
          end
        end
    end
   sum{1,t}=V(:,:,t);
   sum{2,t}=X(:,:,t);
   sum{3,t}=scha(:,:,t);
  end
    %% the acceleration of disturbed vehicle is zero, i.e., v_low
    if(t>dec_t+1)&&(t<=dec_t+stb_t+1)
    for i=1:P      
        for j=1:Mv     
          %lane 3
          if i<=4  % similar to the previous one
            if(j==1)&&(i==1)
            V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
            X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
            vcha(i,j,t)=0;
            scha(i,j,t)=0;
            sdes(i,j,t)=0;
             A(i,j,t)=0;
           % The action of leader in other platoon
           else if(j==1)&&(i~=1)&&(i~=2)
             if((X(i-1,Mv,t-1)-X(i,j,t-1))<=DS)  
             vcha(i,j,t-1)=(V(i,j,t-1)-V(i-1,Mv,t-1));
             scha(i,j,t-1)=(X(i-1,Mv,t-1)-X(i,j,t-1)-L);
             sdes(i,j,t-1)=s0+V(i,j,t-1)*T1+(V(i,j,t-1)*vcha(i,j,t-1))/(2*sqrt(a*b));
             A(i,j,t-1)=a*(1-(V(i,j,t-1)/v0)^4-(sdes(i,j,t-1)/scha(i,j,t-1))^2);

             V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
             X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
             vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
             scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
             sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));

             else if((X(i-1,Mv,t-1)-X(i,j,t-1))>DS)
                if(V(i,j,t-1)>=Vstb)
                    A(i,j,t-1)=0;
                    V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                    X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                    vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                    scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                    sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                else
                    A(i,j,t-1)=a;
                    V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                    X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                    vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                    scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                    sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                end
                 end
             end
                   
            % disturbed vehicle action 
            else if(j==1)&&(i==2)
                    A(2,1,dec_t+1)=0;      % vlow
                    V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                    X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                    vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                    scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                    sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                    A(i,j,t)=A(i,j,t-1);
                end
                 end
            end
        % member vehicle action
        if(j~=1)
            V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
            X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
            vcha(i,j,t)=(V(i,j,t-1)-V(i,j-1,t-1))+(A(i,j,t-1)-A(i,j-1,t-1))*gap;
            scha(i,j,t)=scha(i,j,t-1)+(V(i,j-1,t-1)-V(i,j,t-1))*gap+1/2*(A(i,j-1,t-1)-A(i,j,t-1))*gap^2;
            sdes(i,j,t)=s0+V(i,j,t)*T0+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
            A(i,j,t)=a*(1-(V(i,j,t)/v0)^4-(sdes(i,j,t)/scha(i,j,t))^2);
        end
        
          %lane 1,2,4  Since the deceleration of the disturbed vehicle does not affect the velocity of vehicles in other lanes, the acceleration of vehicle in other lanes is zero.
          else  
            V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
            X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
            vcha(i,j,t)=0;
            scha(i,j,t)=0;
            sdes(i,j,t)=0;
             A(i,j,t)=0;

          end
        end
    end
    sum{1,t}=V(:,:,t);
    sum{2,t}=X(:,:,t);
    sum{3,t}=scha(:,:,t);
    end
   %% disturbed vehicle accelerates
    if(t>dec_t+stb_t+1)&&(t<=N+1)
    for i=1:P      %platoon
        for j=1:Mv     
          if i<=4
            if(j==1)&&(i==1)
            V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
            X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
            vcha(i,j,t)=0;
            scha(i,j,t)=0;
            sdes(i,j,t)=0;
             A(i,j,t)=0;
         % The action of leader in other platoon
         else if(j==1)&&(i~=1)&&(i~=2)
               if((X(i-1,Mv,t-1)-X(i,j,t-1))<=DS)  
                vcha(i,j,t-1)=(V(i,j,t-1)-V(i-1,Mv,t-1));
                scha(i,j,t-1)=(X(i-1,Mv,t-1)-X(i,j,t-1)-L);
                sdes(i,j,t-1)=s0+V(i,j,t-1)*T1+(V(i,j,t-1)*vcha(i,j,t-1))/(2*sqrt(a*b));
                A(i,j,t-1)=a*(1-(V(i,j,t-1)/v0)^4-(sdes(i,j,t-1)/scha(i,j,t-1))^2);

                V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));

               else if((X(i-1,Mv,t-1)-X(i,j,t-1))>DS)  
                       if(V(i,j,t-1)>=Vstb)
                           A(i,j,t-1)=0;  
                           V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                           X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                           vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                           scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                           sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                       else
                           A(i,j,t-1)=a;
                           V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                           X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                           vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                           scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                           sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                       end
                   end
               end
            % disturbed vehicle accelerates
            else if(j==1)&&(i==2)
                    if(V(i,j,t-1)<Vstb)
                      A(2,1,dec_t+stb_t+1)=1.4;
                      V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                      X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                      vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                      scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                      sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                      A(i,j,t)=A(i,j,t-1);
                    else
                      A(2,1,t-1)=0;
                      V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                      X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                      vcha(i,j,t)=(V(i,j,t)-V(i-1,Mv,t));
                      scha(i,j,t)=(X(i-1,Mv,t)-X(i,j,t)-L);
                      sdes(i,j,t)=s0+V(i,j,t)*T1+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                      A(i,j,t)=A(i,j,t-1);
                    end
                  end
               end
           end

               if(j~=1)
                    V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                    X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                    vcha(i,j,t)=(V(i,j,t-1)-V(i,j-1,t-1))+(A(i,j,t-1)-A(i,j-1,t-1))*gap;
                    scha(i,j,t)=scha(i,j,t-1)+(V(i,j-1,t-1)-V(i,j,t-1))*gap+1/2*(A(i,j-1,t-1)-A(i,j,t-1))*gap^2;
                    sdes(i,j,t)=s0+V(i,j,t)*T0+(V(i,j,t)*vcha(i,j,t))/(2*sqrt(a*b));
                    A(i,j,t)=a*(1-(V(i,j,t)/v0)^4-(sdes(i,j,t)/scha(i,j,t))^2);
               end
          else
                %lane 1,2,4  Since the deceleration of the disturbed vehicle does not affect the velocity of vehicles in other lanes, the acceleration of vehicle in other lanes is zero.      
                V(i,j,t)=V(i,j,t-1)+A(i,j,t-1)*gap;
                X(i,j,t)=X(i,j,t-1)+V(i,j,t-1)*gap+1/2*A(i,j,t-1)*gap^2;
                vcha(i,j,t)=0;
                scha(i,j,t)=0;
                sdes(i,j,t)=0;
                A(i,j,t)=0;
          end
        end
    end
    sum{1,t}=V(:,:,t);
    sum{2,t}=X(:,:,t);
    sum{3,t}=scha(:,:,t);
     end
   s=t;
end

%% y position
for tt=1:N+1
    for i=1:1:P
        for j=1:1:Mv
            if i<=4
              Y(i,j,tt)=1.5+width;
            end

            if (i>4)&&(i<=6)
                Y(i,j,tt)=1.5;
            end

            if (i>6)&&(i<=8)
                Y(i,j,tt)=1.5+width*2;
            end

            if (i>8)&&(i<=10)
                Y(i,j,tt)=1.5+width*3;
            end
        end
    end
end

%% position (x,y)
for ttt=1:N+1
    flag_c=0;
    for i=1:1:P;
        for j=1:1:Mv;
            flag_c=flag_c+1;
            position_x(flag_c,ttt)=X(i,j,ttt); % X-axis position for flag_c vehicle in time ttt 

            if i<=4
              position_y(flag_c,ttt)=1.5+width;  % Y-axis position for flag_c vehicle in time ttt 
            end

            if (i>4)&&(i<=6)
                position_y(flag_c,ttt)=1.5;
            end

            if (i>6)&&(i<=8)
                position_y(flag_c,ttt)=1.5+width*2;
            end

            if (i>8)&&(i<=10)
                position_y(flag_c,ttt)=1.5+width*3;
            end
        end
    end
end

 VS2_1=zeros(1,N+1);VS2_2=zeros(1,N+1);VS2_3=zeros(1,N+1);VS2_4=zeros(1,N+1);VS2_5=zeros(1,N+1);VS2_6=zeros(1,N+1);VS2_7=zeros(1,N+1);VS2_8=zeros(1,N+1);
 VS3_1=zeros(1,N+1);VS3_2=zeros(1,N+1);VS3_3=zeros(1,N+1);VS3_4=zeros(1,N+1);VS3_5=zeros(1,N+1);VS3_6=zeros(1,N+1);VS3_7=zeros(1,N+1);VS3_8=zeros(1,N+1);
% velocity
for i=1:N+1
    VS2_1(i)=sum{1,i}(2,1);%  platoon P2 vehicle 1, i.e., disturbed vehicle. P2,1
    VS2_2(i)=sum{1,i}(2,2); 
    VS2_3(i)=sum{1,i}(2,3);
    VS2_4(i)=sum{1,i}(2,4);
    VS2_5(i)=sum{1,i}(2,5);
    VS2_6(i)=sum{1,i}(2,6);
    VS2_7(i)=sum{1,i}(2,7);
    VS2_8(i)=sum{1,i}(2,8);
    
    
    VS3_1(i)=sum{1,i}(3,1);% platoon P3 vehicle 1. P3,1
    VS3_2(i)=sum{1,i}(3,2); 
    VS3_3(i)=sum{1,i}(3,3);
    VS3_4(i)=sum{1,i}(3,4);
    VS3_5(i)=sum{1,i}(3,5);
    VS3_6(i)=sum{1,i}(3,6);
    VS3_7(i)=sum{1,i}(3,7);
    VS3_8(i)=sum{1,i}(3,8);  
end

% H(t)
for ti=1:N+1
    flag0=0;
    for i=1:1:P
        for j=1:1:Mv
            flag0=flag0+1;
            if (sqrt((X(i,j,ti)-X(2,1,ti))^2+(Y(i,j,ti)-Y(2,1,ti))^2)<=R)
               pp=1;
            else
                pp=0;
            end
            H{1,ti}{1,flag0}=pp;% connectivity model
        end
    end
end

for ti=1:N+1
    flag0=0;
    countNumber=0;
    for i=1:1:P*Mv      
        flag0=flag0+1;
        countNumber = countNumber + H{1,ti}{1,flag0};
    end
    Ncs(1,ti)=countNumber;
end

%！！！！！！！！draw velocity！！！！！！！！！！！！%
figure(1);    %  platoon P2 
y=0:gap:N/100;
plot(y,VS2_1,'b',y,VS2_2,'m',y,VS2_3,'g',y,VS2_4,'r',y,VS2_5,'y',y,VS2_6,'--b',y,VS2_7,'c',y,VS2_8,'k');
xlabel('t (second)');
ylabel('Velocity(m/s)');


figure(2);    %  platoon P3
y=0:gap:N/100;
plot(y,VS3_1,'b',y,VS3_2,'m',y,VS3_3,'g',y,VS3_4,'r',y,VS3_5,'y',y,VS3_6,'--b',y,VS3_7,'c',y,VS3_8,'k');
ylim([0,30])
xlabel('t (second)');
ylabel('Velocity(m/s)');
%}

%% mean and variance
% 802.11p parameters
daodalv_0=20;daodalv_1=20;% lambda

slot_time=13*(10^(-6));% slot time

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

A0=0;
A1=AIFSN1-AIFSN0;

pv0=zeros(1,N+1);
pv1=zeros(1,N+1);

w0=zeros(1,N+1);
w1=zeros(1,N+1);

tao0=zeros(1,N+1);
tao1=zeros(1,N+1);
tao=zeros(1,N+1);

PK=zeros(1,N+1);
pb0=zeros(1,N+1);
pb1=zeros(1,N+1);

pb01=zeros(1,N+1);
pb11=zeros(1,N+1);

b0_0=zeros(1,N+1);
b1_0_0=zeros(1,N+1);

g1_1=zeros(1,N+1);
g1_2=zeros(1,N+1);
g1_3=zeros(1,N+1);
g1_4=zeros(1,N+1);
g1_5=zeros(1,N+1);

p0=zeros(1,N+1);
p1=zeros(1,N+1);

p01=zeros(1,N+1);
p11=zeros(1,N+1);

Ts0=zeros(1,N+1);
Ds0=zeros(1,N+1);

Ts1=zeros(1,N+1);
Ds1=zeros(1,N+1);

Ts0_tag=zeros(1,N+1);
Ds0_tag=zeros(1,N+1);
Ts1_tag=zeros(1,N+1);
Ds1_tag=zeros(1,N+1);

for k=1:N+1
 syms z;
 % iterative 
  p0(1,k)=1-exp(-daodalv_0*slot_time);
  p1(1,k)=1-exp(-daodalv_1*slot_time);

  pb0(1,k)=0.2; % the probability that the channel is sensed busy
  pb1(1,k)=0.3;

for u=1:10;
for j=1:10;
b0_0(1,k)=(0.5*(W0_0+1)/(1-pb0(1,k))+(1-p0(1,k))/Pa0)^(-1);
w0(1,k)=b0_0(1,k); % transmission probability for AC0
pv0(1,k)=0;
pv1(1,k)=w0(1,k);

g1_1(1,k)=(1-(pv1(1,k))^(L1+1))/(1-pv1(1,k));
g1_2(1,k)=(W1_0-1)/(2*(1-pb1(1,k)));
g1_3(1,k)=(W1_0*pv1(1,k)*(1-(2*pv1(1,k))^M1))/((1-pb1(1,k))*(1-2*pv1(1,k)));
g1_4(1,k)=((2^(M1-1))*W1_0*(1-(pv1(1,k))^(L1-M1))*pv1(1,k)^(M1+1))/((1-pb1(1,k))*(1-pv1(1,k)));
g1_5(1,k)=(1-p1(1,k))/Pa1;
b1_0_0(1,k)=(g1_1(1,k)+g1_2(1,k)+g1_3(1,k)+g1_4(1,k)+g1_5(1,k))^(-1);
w1(1,k)=g1_1(1,k)*b1_0_0(1,k); % transmission probability for AC1

pb01(1,k)=1-((1-w0(1,k))^(Ncs(1,k)-1))*(1-w1(1,k))^(Ncs(1,k));
pb11(1,k)=1-(((1-w0(1,k))^(Ncs(1,k))) *((1-w1(1,k))^( Ncs(1,k)-1)))^(A1+1); 

piancha1=abs(pb01(1,k)-pb0(1,k));
piancha2=abs(pb11(1,k)-pb1(1,k));

    if(piancha1>0.0000001 && piancha2>0.0000001 )
         pb0(1,k)=pb01(1,k);
         pb1(1,k)=pb11(1,k);
    else
        break;
    end
end
TR=z^Ttr;

% %！！！！！！AC0！！！！！！！！！！！！%
 H0=(1-pb0(1,k))*(z^slot_time)+pb0(1,k)*(z^(Ttr+AIFS0));
 B0_0=0;
 %B0,0(Z)
  for i=0:W0_0-1;
      B0_0=B0_0+(1/W0_0)*(H0)^i;         
  end
  PTs0=TR*B0_0;                         
  diff0_0=diff(PTs0);     
  diff0_1=diff(PTs0,2);      
  
  Ts0(1,k)=subs(diff0_0,z,1); % mean (s)     
  Ds0(1,k)=subs(diff0_1,z,1)+Ts0(1,k)-(Ts0(1,k))^2; 
  
  Ts0_tag(1,k)=Ts0(1,k)*10^3; % mean (ms)
  Ds0_tag(1,k)=sqrt(Ds0(1,k))*10^3; % standard variance (ms)
  
  p01(1,k)=daodalv_0*Ts0(1,k); % server utilization
  piancha3=abs(p01(1,k)-p0(1,k));

 %！！！！！！！！！！！！！！！！！！AC1！！！！！！！！！！！！！！！！%
  H1=(1-pb1(1,k))*(z^slot_time)+pb1(1,k)*(z^(Ttr+AIFS1));
  guodu_jia10=0;guodu_jia11=0;
  
   for i=0:W1_0-1
       guodu_jia10=guodu_jia10+(H1)^i;
   end

  for i=0:W1_1-1
      guodu_jia11=guodu_jia11+(H1)^i;
  end

%  B1,0(z)
    B1_0=(1/W1_0)*guodu_jia10;         
%  B1,1(z)
    B1_1=(1/W1_1)*guodu_jia11;         
%  B1,2(z)
    B1_2=B1_1;       
   
   PTs1=(1-pv1(1,k))*TR*(B1_0+pv1(1,k)*B1_0*B1_1+((pv1(1,k))^2)*B1_0*B1_1*B1_2)+((pv1(1,k))^(L1+1))*B1_0*B1_1*B1_2;
   diff1_0=diff(PTs1);    
   diff1_1=diff(PTs1,2);
   
   Ts1(1,k)=subs(diff1_0,z,1);      
   Ds1(1,k)=subs(diff1_1,z,1)+Ts1(1,k)-(Ts1(1,k))^2;   
   
   Ts1_tag(1,k)=Ts1(1,k)*10^3;
   Ds1_tag(1,k)=sqrt(Ds1(1,k))*10^3;
   
   p11(1,k)=daodalv_1*Ts1(1,k);
   piancha4=abs(p11(1,k)-p1(1,k));
   
   if(piancha3<0.0000001 && piancha4<0.0000001)
        break;
   else     
         p0(1,k)=p01(1,k); 
         p1(1,k)=p11(1,k); 
   end   

end
end
%}

%% performance
%！！！！！！！！！！！！！！delay of AC0、AC1！！！！！！！！！！！！！！！！！！%%
x0=zeros(1,N+1); 
u_0=zeros(1,N+1);
c_0=zeros(1,N+1);
Yt_0=zeros(1,N+1);
Dt_0=zeros(1,N+1);
x0(1,1)=0.00722730738593509;
xt_0=cell(N+1,1);
Xt_0=zeros(1,N+1);
Xt_0(1,1)=x0(1,1);

x1=zeros(1,N+1); 
u_1=zeros(1,N+1);
c_1=zeros(1,N+1);
Yt_1=zeros(1,N+1);
Dt_1=zeros(1,N+1);
x1(1,1)=0.00731440317965779;
xt_1=cell(N+1,1);
Xt_1=zeros(1,N+1);
Xt_1(1,1)=x1(1,1);

% delay
 for i=1:1:N+1;
     u_0(1,i)=1/Ts0(1,i);
     c_0(1,i)=u_0(1,i)*sqrt(Ds0(1,i)); 
     
    % solve differential equation for AC0
    [~,xt_0{i,1}]=ode45('fun',[(i-1)*gap,i*gap],x0(1,i),[],c_0(1,i),u_0(1,i)); 
    xtt_0=xt_0{i,1};
    Xt_0(1,i+1)=xtt_0(length(xtt_0),1); 
    
    % calculate the change rate of average number of packets
    Yt_0(1,i)=-(x0(1,i)+1-sqrt((x0(1,i))^2+2*(c_0(1,i))^2*x0(1,i)+1))/(1-(c_0(1,i))^2)*u_0(1,i)+daodalv_0; 
    
    if i<N+1
    x0(1,i+1)=xt_0{i,1}(length(xtt_0),1); % value of Xt_0(1,i+1)
    end
    % delay
    if(i>1)
         Dt_0(1,i)=Dt_0(1,i-1)+(Yt_0(1,i)/daodalv_0*gap)*(10^3);
    else
        Dt_0(1,1)=(Xt_0(1,1)/daodalv_0)*(10^3);
    end
  
    % ！！！！！AC1！！！！！！%
    u_1(1,i)=1/Ts1(1,i);  
    c_1(1,i)=u_1(1,i)*sqrt(Ds1(1,i)); 
  
    % solve differential equation for AC1
    [~,xt_1{i,1}]=ode45('fun1',[(i-1)*gap,i*gap],x1(1,i),[],c_1(1,i),u_1(1,i));
    xtt_1=xt_1{i,1};
    Xt_1(1,i+1)=xtt_1(length(xtt_1),1);
    
    if i<N+1
    x1(1,i+1)=xt_1{i,1}(length(xtt_1),1);
    end
    % curve fitting to determine utilization function for AC1
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
    Yt_1(1,i)=z;  % calculate the change rate of average number of packets 

    % delay
    if(i>1)
         Dt_1(1,i)=Dt_1(1,i-1)+(Yt_1(1,i)/daodalv_1*gap)*(10^3);
    else
        Dt_1(1,1)=(Xt_1(1,1)/daodalv_1)*(10^3);
    end

  Ts1_tag(1,i)=Ts1(1,i)*10^3;
  Ds1_tag(1,i)=sqrt(Ds1(1,i))*10^3;  
   
 end
  
% ！！！！packet delivery ratio！！！！！%
%！！！！AC0！！！！！！！%
fin_0=zeros(1,N+1);
fout_0=zeros(1,N+1);
G_0=zeros(1,N+1);

for i=1:N+1 
    fin_0(1,i)=daodalv_0; % flow in disturbed vehicle (taeget vehicle) for AC0
    fout_0(1,i)=(x0(1,i)+1-sqrt((x0(1,i))^2+2*(c_0(1,i))^2*x0(1,i)+1))/(1-(c_0(1,i))^2)*u_0(1,i);% flow out disturbed vehicle for AC0
end

%！！！！AC1！！！！！！！%
fin_1=zeros(1,N+1);
fout_1=zeros(1,N+1); 
G_1=zeros(1,N+1);

for i=1:N+1 
    fin_1(1,i)=daodalv_1; % flow in
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
 PDR0=zeros(1,N+1);
 
 PDR1s=zeros((P*Mv),N+1);
 PDR1x=zeros((P*Mv),N+1);
 PDR1=zeros(1,N+1); 
 
 for k=1:1:N+1;
     % calculate the collision probability caused by the exposed terminal
     for r=1:1:(P*Mv);
         if(sqrt((position_x(9,k)-position_x(r,k))^2+(position_y(9,k)-position_y(r,k))^2)<=R)
             h_ir(r,k)=1;
         else
             h_ir(r,k)=0;
         end
            p_exposed(r,k)=h_ir(r,k)*(1-((1-w0(1,k))*(1-w1(1,k)))^(Ncs(1,k)-1)); 
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
                  liancheng(r,j,k)=liancheng(r,j-1,k)*(((1-w0(1,k))*(1-w1(1,k)))^(51*(1-h_ij(j,k))*h_rj(r,j,k)));
             else
                 liancheng(r,1,k)=(((1-w0(1,k))*(1-w1(1,k)))^(51*(1-h_ij(1,k))*h_rj(r,1,k)));
             end
         end
         p_hidden(r,k)=h_ir(r,k)*(1-liancheng(r,(P*Mv),k));
         p_c(r,k)=1-(1-p_exposed(r,k))*(1-p_hidden(r,k)); % collision probability
     end
     
     for r=1:1:(P*Mv);
         if(h_ir(r,k)==1) % within the transmisison range
             if(r~=1) 
                 if(r==9)  % disturbed vehicle (target vehicle)
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
         else  % out of the transmisison range 
             if(r~=1)
                PDR0x(r,k)=PDR0x(r-1,k); % the traffic successfully received by the vehicles keeps the same.
                PDR0s(r,k)=PDR0s(r-1,k); % the traffic that should be received by the vehicles keeps the same.
                
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
     PDR0(1,k)=PDR0s((P*Mv),k)/PDR0x((P*Mv),k); % pacekt delivery ratio for AC0
     PDR1(1,k)=PDR1s((P*Mv),k)/PDR1x((P*Mv),k); % pacekt delivery ratio for AC1    
 end 
 
gap=0.01;
%！！！！！mean and standard variance！！！！！%
figure(3);% mean      
t=0:gap:N/100; % time (s)
plot(t,Ts0_tag,'-m','linewidth',1.5);
hold on
plot(t,Ts1_tag,'--g','linewidth',1.5);
xlabel('t (second)');
ylabel('Mean Service time(ms)'); 
legend('AC0','AC1');

figure(4);  % standard variance
t=0:gap:N/100;
plot(t,Ds0_tag,'-m','linewidth',1.5);
hold on
plot(t,Ds1_tag,'--g','linewidth',1.5);
xlabel('t (second)');
ylabel('Standard Variance (ms)');
legend('AC0','AC1');

%！！！！performance！！%
figure(5);
t=0:gap:N/100;
plot(t,Ncs,'linewidth',1.5);
set(gcf,'unit','centimeters','position',[3 5 10 8.5])
set(gca,'Position',[.15 .15 .8 .75]);
ylim([15,45])
xlabel('t (second)');
ylabel('N_{tr}');

figure(6);
t=0:gap:N/100;
plot(t,Dt_0,'-m','linewidth',1.5);
hold on
plot(t,Dt_1,'--g','linewidth',1.5);
xlabel('t (second)');
ylabel('Packet Delay (ms)');
legend('AC0','AC1');    
 
figure(7);
plot(t, PDR0, '-m','linewidth',1.5);
hold on
plot(t,PDR1,'--g','linewidth',1.5);
% ylim([0.8,1])
xlabel('t (second)');
ylabel('Packet Delivery Ratio');
legend('AC0','AC1');

% ！！！！！！！！Analysis results！！！！！！！！！！！！%
save('Ncs.mat','Ncs');
save('p0.mat','p0');
save('p1.mat','p1');
save('w0.mat','w0');
save('w1.mat','w1');
save('position_x.mat','position_x');
save('position_y.mat','position_y');
save('Ts0_tag.mat','Ts0_tag');
save('Ts1_tag.mat','Ts1_tag');
save('Ds0_tag.mat','Ds0_tag');
save('Ds1_tag.mat','Ds1_tag');
save('Dt_0.mat','Dt_0');        
save('Dt_1.mat','Dt_1');
save('PDR0.mat','PDR0');
save('PDR1.mat','PDR1');

% -------select several points which are used for calculating accuracy of the model------ %
Ts0_tag_theory = zeros(1,24);Ds0_tag_theory = zeros(1,24);Dt_0_theory = zeros(1,24);PDR0_theory = zeros(1,24);
Ts1_tag_theory = zeros(1,24);Ds1_tag_theory = zeros(1,24);Dt_1_theory = zeros(1,24);PDR1_theory = zeros(1,24);

KK=0;SS=3;
for i=1:SS*100:N+1
    KK=KK+1;
   Ts0_tag_theory(1,KK)=Ts0_tag(1,i);
   Ds0_tag_theory(1,KK)=Ds0_tag(1,i);
   Dt_0_theory(1,KK)=Dt_0(1,i);
   PDR0_theory(1,KK)=PDR0(1,i);

   Ts1_tag_theory(1,KK)=Ts1_tag(1,i);
   Ds1_tag_theory(1,KK)=Ds1_tag(1,i);
   Dt_1_theory(1,KK)=Dt_1(1,i);
   PDR1_theory(1,KK)=PDR1(1,i);   
   
end

save('Ts0_tag_theory.mat','Ts0_tag_theory');
save('Ds0_tag_theory.mat','Ds0_tag_theory');

save('Dt_0_theory.mat','Dt_0_theory');
save('PDR0_theory.mat','PDR0_theory');

save('Ts1_tag_theory.mat','Ts1_tag_theory');
save('Ds1_tag_theory.mat','Ds1_tag_theory');

save('Dt_1_theory.mat','Dt_1_theory');
save('PDR1_theory.mat','PDR1_theory');
