%% IMU Visualization Project
clc;
clear;

%Talk to Arduino through com3
s = serialport("COM3",115200);

pause(2);
flush(s);

%% Initilization
% Create Plane Model
figure(1);
filename = 'IMU_Plane_Assembly.obj';
modelData = readObj(filename);
vertices = modelData.v;   % Nx3 matrix of spatial coordinates
faces = modelData.f.v;    % Mx3 matrix defining triangular faces
planePatch = patch('Faces',faces,'Vertices',vertices,'FaceColor',[0.75 0.75 0.75],'EdgeColor','none');
view(3); 
axis equal; 
lighting phong; 
camlight;

% Create skybox
figure(2)
clf                 %what does this do?
axis equal
axis([-1 1 -1 1]);
hold on;
% Background
sky = patch([-1,1,1,-1],[0,0,1,1],'blue');
ground = patch([-1,1,1,-1],[0,0,-1,-1],[153, 102, 51] / 255);
plane = plot([-0.2,0.2,NaN,0,0], [0,0, NaN, -0.05, 0.05], 'w', 'LineWidth',3);
horizon = plot([-2 2],[0 0],'g','LineWidth',2);
% Create telemetry display
telemetry = text(-0.95,0.95,"Calibrating...",...
    'FontSize',10,...
    'Color','white',...
    'VerticalAlignment','top');

% Create Attitude Lines
pitchMarks = -180:20:180;
pitchScale = 0.01;
ladder = gobjects(length(pitchMarks),1);
for i = 1:length(pitchMarks)
    ladder(i) = plot([-0.25 0.25],[pitchMarks(i)*pitchScale pitchMarks(i)*pitchScale],'w','LineWidth',2);
end

skyPoints = [-2 2 2 -2;    % [x1 x2 x3 x4;   <--Column vector form
              0 0 2  2];   %  y1 y2 y3 y4];
groundPoints = [-2 2 2 -2;
                 0 0 -2 -2];
horizonPoints = [-2 2; 
                  0 0];

% Log Values
    timeData = [];
    rollData = [];
    pitchData = [];
    rollAData = [];
    pitchAData = [];
    rollGData = [];
    pitchGData = [];
    yawGData = [];
    GyroXData = [];          
    GyroYData = [];
    GyroZData = [];
    dtData = [];          
    SampleRateData = [];    
    TemperatureData = [];  
    SRavg = [];
    dtavg = [];
    Tempavg = [];
    Timeavg = [];
    startTime = tic;        %Start a timer

%% Data Loop 

testDuration = 60;


while toc(startTime) < testDuration
    %Collect data from port
    line = readline(s);
    values = sscanf(line,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f');
    if numel(values) ~= 13
        continue;   % Ignore startup messages
    end
    roll = values(1);
    pitch = values(2);
    rollA = values(3);
    pitchA = values(4);
    rollG = values(5);
    pitchG = values(6);
    yawG = values(7);
    GyroX = values(8);          %Angular velocity about the x-axis; in deg
    GyroY = values(9);
    GyroZ = values(10);
    dt = values(11);            %Time elapsed between measurements
    SampleRate = values(12);    %Instantaneous Sample Rate [1/dt]
    Temperature = values(13);   %In degC
    disp([roll pitch])

    % Log the Values
    timeData(end+1) = toc(startTime); %log the timer
    rollData(end+1) = roll;
    pitchData(end+1) = pitch;
    rollAData(end+1) = rollA;
    pitchAData(end+1) = pitchA;
    rollGData(end+1) = rollG;
    pitchGData(end+1) = pitchG;
    yawGData(end+1) = yawG;
    GyroXData(end+1) = GyroX;          
    GyroYData(end+1) = GyroY;
    GyroZData(end+1) = GyroZ;
    dtData(end+1) = dt;
    SampleRateData(end+1) = SampleRate;
    TemperatureData(end+1) = Temperature;

    % Create a Moving Average for dt and sample rate
    j=100;
    k=round(j*.9);
    if length(SampleRateData) >= j
        if mod(length(SampleRateData), j) == 0
             SRavg(end+1) = mean(SampleRateData(end-k : end));
             dtavg(end+1) = mean(dtData(end-k : end));
             Tempavg(end+1) = mean(TemperatureData(end-k : end));
             Timeavg(end+1) = timeData(end);
        end
    end

    % 2D Frame Transformation
    Rtwo = [cosd(roll) sind(roll);
         -sind(roll)  cosd(roll)];

    skyRot = Rtwo*skyPoints;
    groundRot = Rtwo*groundPoints;
    horizonRot = Rtwo*horizonPoints;

    skyRot(2,:) = skyRot(2,:) + pitch*pitchScale;
    groundRot(2,:) = groundRot(2,:) + pitch*pitchScale;
    horizonRot(2,:) = horizonRot(2,:) + pitch*pitchScale;

    % 3D Frame Transformation
    c1 = cosd(yawG); s1 = sind(yawG);
    c2 = cosd(pitch); s2 = sind(pitch);
    c3 = cosd(roll); s3 = sind(roll);
    
    R1abt3ax = [c1  s1 0;
               -s1  c1 0;   
                0   0  1];
    R2abt2ax = [c2 0 -s2;
                0  1  0;
                s2 0  c2];
    R3abt1ax = [1  0   0;
                0  c3  s3;
                0 -s3  c3];
    C321 = R3abt1ax * R2abt2ax * R1abt3ax;  % R = Rx*Ry*Rz
    vertices_updated = vertices * C321;
    planePatch.Vertices = vertices_updated; 

    %Updates the Visuals
    horizon.XData = horizonRot(1,:);
    horizon.YData = horizonRot(2,:);
    sky.XData = skyRot(1,:);
    sky.YData = skyRot(2,:);
    ground.XData = groundRot(1,:);
    ground.YData = groundRot(2,:);
    telemetry.String = sprintf("ROLL: %.2f deg\nPITCH: %.2f deg\nSAMPLE RATE: %.2f Hz\nDT: %.5f s\nTEMP: %.2f C", ...
    roll,...
    pitch,...
    SampleRate,...
    dt,...
    Temperature);

    for i = 1:length(pitchMarks)
        y = pitchMarks(i)*pitchScale;
        points = [-0.25  0.25; y  y];
        pointsRot = Rtwo*points;
        pointsRot(2,:) = pointsRot(2,:) + pitch*pitchScale;
        ladder(i).XData = pointsRot(1,:);
        ladder(i).YData = pointsRot(2,:);
    end

    drawnow limitrate

end



%% Attitude Estimate Plotting

figure(3);
clf;

% Roll Plot
subplot(2,1,1); hold on;
plot(timeData, rollData, 'LineWidth',1.5);
plot(timeData, rollAData, 'LineWidth',1.2);
plot(timeData, rollGData, 'LineWidth',1.2);
grid on;
xlabel("Time (s)");
ylabel("Roll (deg)");
title("Roll Attitude Estimate");
legend("Complementary Filter","Accelerometer","Gyroscope");

% Pitch Plot
subplot(2,1,2); hold on;
plot(timeData, pitchData, 'LineWidth',1.5);
plot(timeData, pitchAData, 'LineWidth',1.2);
plot(timeData, pitchGData, 'LineWidth',1.2);
grid on;
xlabel("Time (s)");
ylabel("Pitch (deg)");
title("Pitch Attitude Estimate");
legend("Complementary Filter","Accelerometer","Gyroscope");

figure(4);
% Angular Velocity Plot
hold on;
plot(timeData, GyroXData, 'LineWidth',1);
plot(timeData, GyroYData, 'LineWidth',1);
plot(timeData, GyroZData, 'LineWidth',1);
grid on;
xlabel("Time (s)");
ylabel("Angular Velocity (deg/s)");
title("Angular Velocity Estimate");
legend("Gyro X","Gyro Y","Gyro Z");

% dt and Hertz and temp
% figure(4)
% subplot(1,2,1)
% plot(timeData, dtData, 'LineWidth',1);
% xlabel("Time (s)");
% ylabel("dt");
% title("dt");
% grid on;
% subplot(1,2,2)
% plot(timeData, SampleRateData, 'LineWidth',1);
% xlabel("Time (s)");
% ylabel("Sampling Rate (Hz)");
% title("Instantaneous Sampling Rate");
% grid on;
% 
% figure(5)
% plot(timeData, TemperatureData, 'LineWidth',1);
% xlabel("Time (s)");
% ylabel("Temperature (degC)");
% title("Temperature");
% grid on;

figure(6)
subplot(1,2,1)
plot(Timeavg, dtavg, 'LineWidth',1);
xlabel("Time (s)");
ylabel("dt");
title("dt");
grid on;
subplot(1,2,2)
plot(Timeavg, SRavg, 'LineWidth',1);
xlabel("Time (s)");
ylabel("Sampling Rate (Hz)");
title("Instantaneous Sampling Rate");
grid on;

figure(7)
plot(Timeavg, Tempavg, 'LineWidth',1);
xlabel("Time (s)");
ylabel("Temperature (degC)");
title("Temperature");
grid on;

%% Gyro Bias
GyroXMean = mean(GyroXData);
GyroYMean = mean(GyroYData);
GyroZMean = mean(GyroZData);
disp([GyroXMean GyroYMean GyroZMean]);

%% End Serial Connection
delete(s); 
clear s;