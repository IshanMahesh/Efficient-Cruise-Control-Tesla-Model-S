%%The Efficient Cruise Control
%%

clear ; close all ; clc

%% Scenario

% Road
road_Width              = 11;       % Road width                    [m]
road_Margin             = 2;        % Road margin                   [m]
road_Dist_Analysis      = 200;      % Road distance analysis        [m]
% Vehicle
vehicle_Length          = 11.15;     % Length of the vehicle         [m]
vehicle_Width           = 2.8;     % Width of the vehicle          [m]
vehicle_Initial_Speed   = 300/1.7;   % Initial speed of the vehicle  [m/s]
vehicle_Mass            = 1000;     % Mass of the vehicle           [kg]
vehicle_Area            = 2.5;      % Frontal area of the vehicle   [m2]
vehicle_Cd              = 0.5;     % Drag coefficient              [-]
air_Density             = 2;        % Air density                   [kg/m3]

% Lumped air drag coefficient [N(s/m)2]
C = 0.5 * vehicle_Area * vehicle_Cd * air_Density;   

% Vehicle struct
vehicle.C = C;
vehicle.M = vehicle_Mass;

% Parameters
Tf      = 60;                       % Final time                    [s]
FR      = 30;                       % Frame rate                    [fps]
dt      = 1/FR;                     % Time resolution               [s]
Tspan   = linspace(0,Tf,Tf*FR);     % Time                          [s]

%% Simulation

% Initial conditions
z0 = [0 vehicle_Initial_Speed]; 

% Integration
options = odeset('RelTol',1e-6);
[t_OUT,z_OUT] = ode45(@(t,z) vh_dyn(t,z,vehicle),Tspan,z0,options);

% States
vehicle_position    = z_OUT(:,1);
vehicle_speed       = z_OUT(:,2);


vh_acc         = zeros(1,length(t_OUT));
frc_long          = zeros(1,length(t_OUT));
spd_ref           = zeros(1,length(t_OUT));
for i=1:length(t_OUT)
    [dz,F_l,V_r]    = vh_dyn(t_OUT(i),z_OUT(i,:),vehicle);
    vh_acc(i)  = dz(2);
    frc_long(i)   = F_l;
    spd_ref(i)    = V_r;
end

%% Results

figure
set(gcf,'Position',[50 50 640 640])
    
% Create a video writer object
v = VideoWriter('cruise_control.mp4','MPEG-4');
v.Quality = 100;
open(v);

for i=1:length(t_OUT)
    subplot(3,2,1)
        hold on ; grid on ; box on
        set(gca,'xlim',[0 t_OUT(end)],'ylim',[0 1.2*max(vehicle_position)])
        cla 
        plot(t_OUT,vehicle_position,'b')
        plot([t_OUT(i) t_OUT(i)],[0 1.2*max(vehicle_position)],'k--') 
        xlabel('Time [s]')
        ylabel('Position [m]')
        title('Position')
    subplot(3,2,2)
        hold on ; grid on ; box on
        set(gca,'xlim',[0 t_OUT(end)],'ylim',[0 1.2*max(vehicle_speed)])
        cla 
        plot(t_OUT,spd_ref,'k')
        plot(t_OUT,vehicle_speed,'b')
        plot([t_OUT(i) t_OUT(i)],[0 1.2*max(vehicle_speed)],'k--') 
        xlabel('Time [s]')
        ylabel('Speed [m/s]')
        title('Speed (Black=Reference, Blue=Actual)')
    subplot(3,2,3)
        hold on ; grid on ; box on
        set(gca,'xlim',[0 t_OUT(end)],'ylim',[min(vh_acc)-1 max(vh_acc)+1])
        cla 
        plot(t_OUT,vh_acc,'b')
        plot([t_OUT(i) t_OUT(i)],[min(vh_acc)-1 max(vh_acc)+1],'k--') 
        xlabel('Time [s]')
        ylabel('Acceleration [m/s2]')
        title('Acceleration')
    subplot(3,2,4)
        hold on ; grid on ; box on
        set(gca,'xlim',[0 t_OUT(end)],'ylim',[min(frc_long)-500 max(frc_long)+500])
        cla 
        plot(Tspan,frc_long,'b')
        plot([t_OUT(i) t_OUT(i)],[min(frc_long)-500 max(frc_long)+500],'k--') 
        xlabel('Time [s]')
        ylabel('Lon. force [N]')
        title('Longitudinal force')
    subplot(3,2,5:6)
        hold on ; axis equal ; box on
        cla 
        % Position of the vehicle at current instant [m]
        vehicle_position_inst = vehicle_position(i);

        sideMarkingsX = [vehicle_position_inst-road_Dist_Analysis/2 vehicle_position_inst+road_Dist_Analysis/2];
        set(gca,'xlim',sideMarkingsX,'ylim',[-road_Width/2-road_Margin +road_Width/2+road_Margin])

        plot(sideMarkingsX,[+road_Width/2 +road_Width/2],'k--') % Left marking
        plot(sideMarkingsX,[-road_Width/2 -road_Width/2],'k--') % Right marking

        % Dimensions
        vehicle_dimension_X = [vehicle_position_inst vehicle_position_inst vehicle_position_inst-vehicle_Length vehicle_position_inst-vehicle_Length];
        vehicle_dimension_Y = [+vehicle_Width/2 -vehicle_Width/2 -vehicle_Width/2 +vehicle_Width/2];
        % Plotting
        fill(vehicle_dimension_X,vehicle_dimension_Y,'r')
        
        xlabel('Lon. distance [m]')
        ylabel('Lat. distance [m]')
    
    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);

%% Auxiliary functions

function [dst,f_lon,v_ref] = vh_dyn(t,st,vh)
    
    % Parameters
    M = vh.M;              % Mass of the vehicle           [kg]
    C = vh.C;              % Lumped air drag coefficient   [N(s/m)2]

    % States
%     X = states(1);
    V = st(2);

    % Drag resistance
    Dx = C*V^2;
    
    % Reference speed [m/s]
    if t < 20
        v_ref   = 25; 
    elseif t < 40
        v_ref   = 10; 
    else
        v_ref   = 20;
    end
    
    % Cruise controller
    Kp = 500;
    f_lon  = Kp*(v_ref - V) + C*v_ref^2;
    
    % Dynamics
    dst(1,1) = V;
    dst(2,1) = (f_lon - Dx)/m;
    
end
