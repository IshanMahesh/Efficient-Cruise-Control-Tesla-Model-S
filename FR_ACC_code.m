% Feedback ACC model
% A platoon of vehicles where leader behaviour is exogenous
function FR_ACC_code
clear all; close all;

% General simulation controller parameters
s0 = 3;         % minimum gap at standstill [m]
td = 1.2;       % desired time gap [s]
l = 4;          % vehicle length [m]
r = 150;        % on-board sensor range [m]
u_max = 1.5;    % maximum acceleration [m/s^2]
u_min = -8;     % minimum acceleration [m/s^2]
T = 200; % Simulation time [s]
dt_sim = 0.1; % simulation time step [s]
n_sim = T/dt_sim; % number of simulation steps
n_f = 1; % number of followers in the platoon, excluding the leader



% Intake of case and delay
cs=input('what is the case (choose a number 1-5, 1:stop and go, 2:normal, 3: emergency braking, 4:cut in, 5: user defined): '); 
rt=input('what is the sensor delay( >0, in terms of simulation step, i.e entry 1 means sensor delay of 0.1s with time step 0.1s):  ');
ad=input('what is the adaptation delay( >0, in terms of simulation step, i.e entry 1 means sensor delay of 0.1s with time step 0.1s):  ');

% Reference FRACC parameters (Moon et al. 2009)
tau=1.2;                  % FRACC linear coefficient cd
Tdelay = 0.5;             % time delay [s] parameter in (Moon FRACC)
Treaction = 0.8;          % Reaction time [s] parameter in (Moon FRACC)
mu = 1;                   % coeficient of friction in (Moon FRACC)
ITTC =zeros(n_f,n_sim);   % inverse travel time (Moon FRACC)
CM =zeros(n_f,n_sim);     % Control Mode (Moon FRACC)
CD =zeros(n_f,n_sim);     % Equilibrium distance (Moon FRACC)
dbr =zeros(n_f,n_sim);    % FRACC braking distance
dw =zeros(n_f,n_sim);     % FRACC warning distance
tst =zeros(n_f,n_sim);    % FRACC test
X = zeros(n_f,n_sim);     % warning index (Moon FRACC)
AF = zeros(n_f,n_sim);    % Acceleration of two followers  (close loop)
VF = zeros(n_f,n_sim);    % Speed of two followers (close loop)
XF = zeros(n_f,n_sim);    % Position of two followers (close loop)
S = zeros(n_f,n_sim);     % Gap of two followers (close loop)
DV = zeros(n_f,n_sim);    % Relative speed of two followers (close loop)
ve = zeros(n_f,n_sim);
test1= zeros(n_f,n_sim);
test2= zeros(n_f,n_sim);
TEM = zeros(n_f,5);
TTM = zeros(n_f,1);
JM = zeros(n_f,n_sim);
JEM = zeros(n_f,5);
TJM = 0;
ASM= 0;
%proposed controller parameters
% k1 =0.1 , k2 = 5.4 for better collision avoidance performance
k1 = 0.18;      % weight on equilibrium speed
k2 = 1.93;      % weight on realtive speed

T1= zeros(n_f,n_sim);    % Test term1
T2 = zeros(n_f,n_sim);    % Test term2
T3 = zeros(n_f,n_sim);   % Test term3
AFp = zeros(n_f,n_sim);    % Acceleration of two followers  (close loop)
VFp = zeros(n_f,n_sim);    % Speed of two followers (close loop)
XFp = zeros(n_f,n_sim);    % Position of two followers (close loop)
DVp = zeros(n_f,n_sim);    % Relative speed of two followers (close loop)
Sp = zeros(n_f,n_sim);     % Gap of two followers (close loop)
TEP = zeros(n_f,5);
TTP=zeros(n_f,1);
JP = zeros(n_f,n_sim);
JEP = zeros(n_f,5);
TJP =  0;
ASP = 0;            

% pre-allocation of close-loop variables
AL = zeros(1,n_sim);    % Acceleration of leader (close loop)
vleader = zeros(1,n_sim); % Speed of leader (close loop)
xleader = zeros(1,n_sim); % Speed of leader (close loop)


% SET UP INITIAL CONDITIONS
% start velocity of the leader
 switch cs
        case 1
            vleader(1) =5.5;
        case 2
            vleader(1) =20;
        case 3
             vleader(1) =22.2;
        case 4
             vleader(1) =22.2;
        case 5
             vleader(1) =0;
         case 0
            vleader(1) =input('what is the initial velocity of leader in m/s ');
 end
 
% FRACC Initial state
xleader(1) = 500;                   % Initial position of leader [m]
VF(:,1) = vleader(1);               % initial velocity of follower
S(:,1) = vleader(1) * td + s0;      % starting with equilibrium gap
XF(1,1) = xleader(1) - S(1,1)-l;    % position of follower 1
XF(2:n_f,1) = XF(1:n_f-1,1) - S(2:n_f,1) - l; % positions of subsequent folowers

% Proposed controller intial state
VFp(:,1) = vleader(1);    % initial velocity of follower
Sp(:,1) = vleader(1) * td + s0; % starting with equilibrium gap
XFp(1,1) = xleader(1) - Sp(1,1)-l;  % position of follower 1
XFp(2:n_f,1) = XFp(1:n_f-1,1) - Sp(2:n_f,1) - l; % positions of subsequent folowers

% main loop for simulation
for k = 1:n_sim-1
    t = k*dt_sim;
    
    % Predefined leader acceleration( collision ):
    switch cs
        case 1 % Stop and Go (make starting vel= 5.5)
             if (t > 3) && (t <= 4)
                AL(k) =  0;
           elseif (t > 5) && (t <= 20)
                AL(k) = -0.38;
                k;
           elseif (t > 40) && (t <= 80)
                AL(k) = 0.39;
                k;
            elseif (t > 130) && (t <= 170)
                AL(k) = -0.39;
                k;
             else 
                AL(k) = 0;
             end
        case 2 % normal ACC Scenario(make starting velocity 20)
            if (t > 3) && (t <= 4)
                AL(k) =  0;
               
            elseif (t > 70) && (t <= 80)
                AL(k) = 0.5;
                k;
            elseif (t > 140) && (t <= 160)
                AL(k) = -0.5;
                k;
             else 
                AL(k) = 0;
            end
        case 3 % Emergency braking (make starting velocity 22.2)
            if (t > 3) && (t <= 4)
                AL(k) =  0;
            elseif (t > 60) && (t <= 70)
                AL(k) = -4.45;
                k;
             else 
                AL(k) = 0;
            end
         case 4 % CUT IN (make starting velocity 22.2)
            if (t > 3) && (t <= 4)
                AL(k) =  0;
                      
            elseif (t == 60) 
                xleader(k)= xleader(k)-(Sp(1,k)./2);
                Sp(1,k) = Sp(1,k)./2;
                S(1,k) = S(1,k)./2;
            else 
                AL(k) = 0;
            end
          case 5 % add your own code scenario here
           
           
    end
    
    
    %% First forward;
    ve(:,k) = (Sp(:,k) - s0)/td; % equilibrium speed 
    ve(:,k) = max(ve(:,k),0); 
    
    %SIMULATION OF FRACC-moon et.al.2009
    CD(1,k) = s0 + (tau* vleader(k));
    % control mode decision
    if k<(rt)&&(k~=1)
    AF(:,k)= AF(:,k-1);
    elseif (k<n_sim-rt+1)
    ITTC(:,k) = (-DV( : ,k))./S( : ,k);
    for j=1:n_f
        P = FRACCparameters(VF(j,k));
        if j>1
            CD(j,k) = s0 + (tau* VF(j-1,k));
        end
        dbr(j,k) = (-DV(j,k)*Tdelay)+ (mu*((VF(j,k).^2-(VF(j,k)+DV(j,k)).^2))/(2*P(1)));
        dw(j,k) = (-DV(j,k)*Tdelay)+ (mu*((VF(j,k).^2-(VF(j,k)+DV(j,k)).^2))/(2*P(1)))+ (VF(j,k)*Treaction);
        X(j,k) = (S( j,k)- dbr(j,k))/(dw(j,k) - dbr(j,k));
        tst(j,k) =  CD(j,k);    
        if (X(j,k) > 1.19) && (ITTC(j,k)< 0.21) %CONTROL MODE 1
        CM (j,k)=1;
        AF(j,k+rt) = (min(P(1),max((-P(3)*(CD(j,k)- S(j,k))- P(4)*DV(j,k)),P(2))))/ad +(AF(:,k+rt-1)*(1-(1/ad)));
        test1(j,k)= P(3);
        test2(j,k)= P(4);  
        elseif(X(j,k) < 0.81) && (ITTC(j,k)> 0.49) %CONTROL MODE 3
        CM (j,k)=3;
        fprintf('Collision avoidance activated by %d at %d \n',j,(k*0.1));
        AF(j,k+rt)  = (P(5)*(f1(X(j,k)))+ P(6)*f2(ITTC(j,k)))/ad +(AF(:,k+rt-1)*(1-(1/ad)));
        test1(j,k)= P(3);
        test2(j,k)= P(4); 
        elseif (X(j,k)< 1.19)||(ITTC(j,k)>0.21);   %CONTROL MODE 2
        CM (j,k)=2;
        AF(j,k+rt) = (max((-P(3)*(CD(j,k)- S(j,k))- (P(4)*DV(j,k))),-4))/ad +(AF(:,k+rt-1)*(1-(1/ad)));
        test1(j,k)= P(3);
        test2(j,k)= P(4);         
        end
                         
    end
    end
       
    %SIMULATION OF THE PROPOSED CONTROLLER
    if k<(rt)&&(k~=1)
        AFp(:,k)= AFp(:,k-1);
    elseif (k<n_sim-rt+1)
        T2(:,k) = k2*DVp(:,k).*((-1./(1+(1.*exp(-(0.01.*Sp(:,k)))))+1));
        AFp(:,k+rt)  = ((k1*(min((Sp(:,k)-s0),36)- VFp(:,k)*td)+ T2(:,k))/ad)+(AFp(:,k+rt-1)*(1-(1/ad)));
        T1(:,k+rt)= DVp(:,k)<=0;
        T3(:,k+rt) =  k2*((-1/(1+exp(-0.01.*(Sp(:,k)-3))))+1);
    end
          
     
    % Limited acceleration
    AF(:,k) = max(AF(:,k), u_min);
    AF(:,k) = min(AF(:,k), u_max);
    AFp(:,k) = min(AFp(:,k), u_max);
    % Limit acceleration such that v >= 0
    AF(:,k) = max(AF(:,k),-VF(:,k)/dt_sim);
    AFp(:,k) = max(AFp(:,k),-VFp(:,k)/dt_sim);
    AL(k)=max(AL(k),-vleader(k)/dt_sim);
    
    switch cs
        case 1 % Stop and Go (make starting vel= 5.5)
            if(t>3) 
               JM(:,k) = abs(AF(:,k)- AF(:,k-1));
               JP(:,k) = abs(AFp(:,k)- AFp(:,k-1));
            end            
            if (t > 20) && (t <= 50)
                if(abs(AL(:,k)- AFp(:,k))> 0.01)
                 TEP(:,1)= TEP(:,1)+ dt_sim;
                 JEP(:,1) = JEP(:,1)+JP(:,k);
                end
                if(abs(AL(:,k)- AF(:,k))> 0.01)
                 TEM(:,1)= TEM(:,1)+ dt_sim;
                 JEM(:,1) = JEM(:,1)+JM(:,k);
                end
            elseif (t > 80) && (t <= 100)
                 if(abs(AL(:,k)- AFp(:,k))> 0.01)
                 TEP(:,2)= TEP(:,2)+ dt_sim;
                  JEP(:,2) = JEP(:,2)+JP(:,k);
                end
                 if(abs(AL(:,k)- AF(:,k))> 0.01)
                 TEM(:,2)= TEM(:,2)+ dt_sim;
                 JEM(:,2) = JEM(:,2)+JM(:,k);
                end 
            elseif (t > 170) && (t <= 200)
               %if(abs(DVp(:,k))> 0.1)
                if(abs(AL(:,k)- AFp(:,k))> 0.01)
                 TEP(:,3)= TEP(:,3)+ dt_sim;
                 JEP(:,3) = JEP(:,3)+JP(:,k);
                end
                %if(abs(DV(:,k))>0.1)
                 if(abs(AL(:,k)- AFp(:,k))> 0.01)
                 TEM(:,3)= TEM(:,3)+ dt_sim;
                 JEM(:,3) = JEM(:,3)+JM(:,k);
                end
            end
            if(XF(:,k)<1600)
                TTM(:,1) = t;
            end
            if(XFp(:,k)<1600)
                TTP(:,1)= t;
            end
            TJP =  TJP + JP(:,k);
            TJM = TJM +JM(:,k);
            ASM = ASM +DV(:,k)/S(:,k);
            ASP = ASP +DVp(:,k)/Sp(:,k);
           
        case 2 % normal ACC Scenario(make starting velocity 20)
                          
          if(t>3)
            JM(:,k) = abs(AF(:,k)- AF(:,k-1));
            JP(:,k) = abs(AFp(:,k)- AFp(:,k-1));
          end
          if (t > 80) && (t <= 90)
                if(abs(DVp(:,k))> 0.1)
                 TEP(:,1)= TEP(:,1)+ dt_sim;
                 JEP(:,1) = JEP(:,1)+JP(:,k);
                end
                if(abs(DV(:,k))>0.1)
                 TEM(:,1)= TEM(:,1)+ dt_sim;
                 JEM(:,1) = JEM(:,1)+JM(:,k);
                end   
          elseif (t > 160) && (t <= 170)
                if(abs(DVp(:,k))> 0.1)
                 TEP(:,2)= TEP(:,2)+ dt_sim;
                 JEP(:,2) = JEP(:,2)+JP(:,k);
                end
                if(abs(DV(:,k))>0.1)
                 TEM(:,2)= TEM(:,2)+ dt_sim;
                 JEM(:,2) = JEM(:,2)+JM(:,k);
                end  
           end
            if(XF(:,k)<4200)
                 TTM(:,1) = t;
            end
            if(XFp(:,k)<4200)
                TTP(:,1)= t;
            end
                     
            TJP =  TJP + JP(:,k);
            TJM = TJM +JM(:,k);
            ASM = ASM +DV(:,k)/S(:,k);
            ASP = ASP +DVp(:,k)/Sp(:,k);
            
        case 3 % Emergency braking (make starting velocity 22.2)
           if(t>3)
            JM(:,k) = abs(AF(:,k)- AF(:,k-1));
            JP(:,k) = abs(AFp(:,k)- AFp(:,k-1));
           end
           if (t > 70) && (t <= 80)
                if(abs(DVp(:,k))> 0.1)
                 TEP(:,1)= TEP(:,1)+ dt_sim;
                end
                if(abs(DV(:,k))>0.1)
                 TEM(:,1)= TEM(:,1)+ dt_sim;
                end   
           end
            TJP =  TJP + JP(:,k);
            TJM = TJM +JM(:,k);
            ASM = ASM +S(:,k);
            ASP = ASP +Sp(:,k);
            
         case 4 % Cut in (make starting velocity 22.2)
          if(t>3)
                JM(:,k) = abs(AF(:,k)- AF(:,k-1));
                JP(:,k) = abs(AFp(:,k)- AFp(:,k-1));
          end
          if (t> 60) && (t <= 90)
                if(abs(DVp(:,k))> 0.1)
                 TEP(:,1)= TEP(:,1)+ dt_sim;
                 JEP(:,1) = JEP(:,1)+JP(:,k);
                end
                if(abs(DV(:,k))>0.1)
                 TEM(:,1)= TEM(:,1)+ dt_sim;
                 JEM(:,1) = JEM(:,1)+JM(:,k);
                end                             
          end
            TJP =  TJP + JP(:,k);
            TJM = TJM +JM(:,k);
            ASM = ASM +S(:,k);
            ASP = ASP +Sp(:,k);
    end
    
    % Update speed and position of leader for the next time step
    vleader(k+1) = vleader(k) + dt_sim * AL(k);
    xleader(k+1) = xleader(k) + dt_sim * vleader(k) + 0.5 * AL(k)* dt_sim^2;
    
   % Update speed and position of leader for the next time step FRACC-Moon
    VF(:,k+1) = VF(:,k) + AF(:,k) * dt_sim;
    XF(:,k+1) = XF(:,k) + VF(:,k) * dt_sim + 0.5 * AF(:,k) * dt_sim^2;
    S(1,k+1) = S(1,k) + DV(1,k) * dt_sim + 0.5 * (AL(k)-AF(1,k)) * dt_sim^2;
    DV(1,k+1) = DV(1,k) + (AL(k)-AF(1,k)) * dt_sim;
    
    % Update speed and position of leader for the next time step-  proposed
    VFp(:,k+1) = VFp(:,k) + AFp(:,k) * dt_sim;
    XFp(:,k+1) = XFp(:,k) + VFp(:,k) * dt_sim + 0.5 * AFp(:,k) * dt_sim^2;
    Sp(1,k+1) = Sp(1,k) + DVp(1,k) * dt_sim + 0.5 * (AL(k)-AFp(1,k)) * dt_sim^2;
    DVp(1,k+1) = DVp(1,k) + (AL(k)-AFp(1,k)) * dt_sim;
    
    
    for i_f = 2: n_f
        S(i_f,k+1) = S(i_f,k) + DV(i_f,k) * dt_sim + 0.5 * (AF(i_f-1,k)-AF(i_f,k)) * dt_sim^2;
        DV(i_f,k+1) = DV(i_f,k) + (AF(i_f-1,k)-AF(i_f,k)) * dt_sim;
    end
    for i_f = 2: n_f
        Sp(i_f,k+1) = Sp(i_f,k) + DVp(i_f,k) * dt_sim + 0.5 * (AFp(i_f-1,k)-AFp(i_f,k)) * dt_sim^2;
        DVp(i_f,k+1) = DVp(i_f,k) + (AFp(i_f-1,k)-AFp(i_f,k)) * dt_sim;
    end
    
    
end
mjm = max(JM(:,:));
mjp = max(JP(:,:));
 ASM = ASM/n_sim;
 ASP = ASP/n_sim;
% save results



%% Plot results
taxis = 0:dt_sim:T-dt_sim;
set(0,'defaultAxesFontName', 'Times New Roman')
set(0,'DefaultLineLineWidth',1)
figure;
subplot(111)
plot(taxis, AL(1,:),'k--', taxis, AF(:,:),'b-.', taxis, AFp(:,:),'r');
set(gca,'fontsize',15)
legend('Leader','MS-FRACC','P-FRACC')
xlabel({'time [s]','(a)'}); ylabel('achieved acceleration [m/s^2]');
figure;
subplot(111)
plot(taxis, vleader(1,:),'k--', taxis, VF(:,:),'b-.', taxis, VFp(:,:),'r');
set(gca,'fontsize',15)
xlabel({'time [s]','(b)'}); ylabel('velocity [m/s]');
legend('Leader','MS-FRACC','P-FRACC')
end
function [a]= FRACCparameters(y)
    if y <= 11.11
    a(1) = 1.5;         % max acceleration 
    a(2) = -2;          % min acceleration
    a(3) = 0.35;        % k1
    a(4) = -1.2;        % k2
    a(5) = 1;           % w1
    a(6) = 0;           % w2
    elseif y >= 19.44
    a(1) = 1;            % max acceleration 
    a(2) = -1;           % min acceleration
    a(3) = 0.2;          % k1
    a(4) = -0.85;        % k2
    a(5) = 0;            % w1
    a(6) = 1;            % w2
    else
    a(1) = ((1-1.5)/(19.44-11.11)*(y-11.11))+1.5;            % max acceleration 
    a(2) = (((-1)-(-2))/(19.44-11.11)*(y-11.11))+(-2);       % min acceleration
    a(3) = (((0.2)-(0.35))/(19.44-11.11)*(y-11.11))+(0.35);  % k1
    a(4) = (((-0.85)-(-1.2))/(19.44-11.11)*(y-11.11))+(-1.2);% k2
    a(5) = (((0)-(1))/(19.44-11.11)*(y-11.11))+(1);          % w1
    a(6) = (((1)-(0))/(19.44-11.11)*(y-11.11))+(0);          % w2
    end
end


function [a]= f1(y)
if y <= 0.3
    a = -8;
elseif y <= 0.65
    a = ((y-0.3)/(0.3-0.65)*((-8)-(-6)))+ (-8);
elseif y <= 0.81
    a = ((y-0.65)/(0.65-0.81)*((-6)-(-4)))+ (-6);
elseif y <= 1.19
    a = ((y-0.81)/(0.81-1.19)*((-4)-(-2)))+ (-4);
else
    a = 0;
end
end
function [a]= f2(y)
if y <= 0.21
    a = -2;
elseif y <= 0.49
    a = ((y-0.21)/(0.21-0.49)*((-2)-(-4)))+ (-2);
elseif y <= 0.68
    a = ((y-0.49)/(0.49-0.68)*((-4)-(-6)))+ (-4);
elseif y <= 1.5
    a = ((y-0.68)/(0.68-1.5)*((-6)-(-8)))+ (-6);
else 
    a = 0;
end
end