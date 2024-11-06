clc;
clear;
close all;

% Raspberry Pi SSH parameters
% Replace with your Raspberry Pi's IP address
raspberryPiIP = '192.168.1.10';
username = 'labraspberry'; % Replace with your Raspberry Pi username
% Replace with the name of the script you want to stop
scriptName = 'StepMotor.py'; % Replace with the actual script name
% Construct the SSH command to stop the process
sshStopCommand = sprintf('ssh %s@%s "pkill -f %s"', username,...
    raspberryPiIP, scriptName);
system(['start /B ' sshStopCommand]);
% Command to run the Python script in the background
command = 'cd ~/StepMotor && python StepMotor.py';
% Construct the SSH command
sshCommand = sprintf('ssh %s@%s "%s"', username, raspberryPiIP, command);
% Execute the SSH command in the ...
% background using the start command (Windows)
system(['start /B ' sshCommand]);


% Construct NI daq Object and Channels
dq = daq("NI");
Sampling_rate = input("Please specify the sampling rate: ");
dq.Rate = Sampling_rate;
N_channels = input("Please specify the number of DAQ channels: ");
Force_channels = struct();
Accelerometer_channels = struct();
for i = 1:N_channels
    addinput(dq, "cDAQ9185Mod4", strcat("ai", num2str(i - 1)), "Voltage");
    Channel_type = input(sprintf("Please Enter F as Force Channel..." + ...
        " and Ac as Accelerometer for the channel %d:  ", i), 's');

    switch(Channel_type)
        case 'F'
            Force_channels.(strcat("cDAQ9185Mod4_ai", num2str(i - 1)))= ...
                input(sprintf("What is the channel %d sensitivity: ", i));
        case 'Ac'
            Accelerometer_channels.(strcat("cDAQ9185Mod4_ai", ...
                num2str(i - 1))) = ...
            input(sprintf("What is the channel %d sensitivity: ", i));
        otherwise
            error("Invalid channel type entered...." + ...
                " Please use 'F' for Force or 'Ac' for Accelerometer.");
    end
end

Force_channels_names = fieldnames(Force_channels);
Accelerometer_channels_names = fieldnames(Accelerometer_channels);

% Establish Serial Communication
Serial = serialport("COM3", 57600);
configureTerminator = 'LF';
fopen(Serial);

% variable Declaration. Do Not Change These Variables
cdaqTrig = false;
nominal_hit = false;
calibration = false;
FM = 0;
need_calibration = input("Do you need Calibration? Yes/ No ", 's');
fwrite(Serial,need_calibration, 'char');

% Calibration process:
while ~calibration
    if need_calibration == "Yes"
        while true
            disp("Calibration Process; Enter the number ..." + ...
                " associated with your choice. 1:" + ...
                " Move Forward , 2 - MoveBackward , 3 -Ready");
            commmand = input("Choice: ");
            switch(commmand)
                case 1
                    fwrite(Serial,1,"uint16");
                case 2
                    fwrite(Serial,2,'uint16');
                case 3
                    fwrite(Serial,3,'uint16');
                    break;
            end
        end
        calibration = true;
    end
    if need_calibration == "No"
        break;
    end
end
pause(1);
% Specify Number of Experiments
N = input("Please Enter the number of Experiments: ");  
fwrite(Serial,N,"uint16");
pause(1)
% Specify Number of Iterations
iter = input("Please Enter the number of Iterations: ");  
fwrite(Serial,iter,"uint16");
pause(1)
% Specify Error rate
error_rate = input("Please Enter the error rate (0.5 to 5): ");  
fwrite(Serial,error_rate,"single");
pause(1)
% Specify the hammer tip type
choice = input("Which hammer Tip are you using?", 's');
fwrite(Serial, choice, 'char');
% Measurement Time Constant
tDur = input("Please specify the measurement time: ");

Data = struct('Time',struct(),'Force', struct(), 'Acc', struct());
Data.Time = cell(N,1);

for i = 1:N
    DF = input("Please Enter the desired force:");
    fwrite(Serial, DF, 'uint16');
    cdaqTrig = false;
    
    while ~cdaqTrig
        a = read(Serial, 1, "string");
        Trigger = str2double(a);
        
        if Trigger ~= 0
            acquired_data = read(dq, seconds(10 + tDur));
            t = seconds(acquired_data.Time);
            for j = 1:numel(Force_channels_names)
                name = Force_channels_names{j};
                Force_data = acquired_data.(name) * 1000 /...
                    Force_channels.(name);
                [~, IFx] = max(Force_data);
                tHit = t(IFx);
                tStart = tHit - 0.1;
                tA = sum(t <= tStart);

                t = t(tA:end) - t(tA);
                Data.Force.(name){i} = Force_data(tA:end);
                Data.Time{i} = t;
                Force = Data.Force.(name){i};
                FM = max(Force)
                error = (FM - DF)/DF
                figure(j);
                plot(t, Force,'k');
                xlabel('Time [s]');
                ylabel('Force [N]');
            end

            for k = 1:length(Accelerometer_channels_names)
                name = Accelerometer_channels_names{k};
                Acc_data = acquired_data.(name) * 1000 /...
                    Accelerometer_channels.(name);
                Data.Acc.(name){i} = Acc_data(tA:end,:);
                figure(j+k);
                plot(t, Data.Acc.(name){i}, 'k');
                xlabel('Time [s]');
                ylabel("Acc [m/s^2]");
                title(name);
            end
            
            fwrite(Serial, FM, 'single');
            Trigger = 0;
        end
        
        error_acceptance_rate = false;
        while ~error_acceptance_rate
            b = str2double(read(Serial, 1, 'string'));
            if b == 1
                error_acceptance_rate = true;
                cdaqTrig = true;
            end
            if b == 2
                error_acceptance_rate = true;
            end
        end
    end
    % %Repeatability test  % This part is only for validating system
    % repeatability. Comment or Delete.
    
    %     repeat = input("Send Yes if you want to check repeatability:...
    % ",'s');
    %     close all;
    %     fwrite(Serial, repeat, 'char');
    %     flush(dq);
    %     c=0;
    %     nominal_hit = false;
    %     while ~nominal_hit
    %         c = str2double(read(Serial,1,'string'));
    %         if c ==1
    %             data= read(dq,seconds(10+tDur));
    %             t = seconds(data.Time);
    %             repeat_Force_data = struct();
    %
    %             for j = 1:numel(Force_channels_names)
    %                 name = Force_channels_names{j};
    %                 repeat_Force_data.(name) = data.(name) * 1000 /...
    % Force_channels.(name);
    %                 [~, IFx] = max(repeat_Force_data.(name));
    %                 tHit = t(IFx);
    %                 tStart = tHit- 0.5;
    %                 tA = sum(t <= tStart);
    %
    %                 t = t(tA:end)-t(tA);
    %                 Force =  repeat_Force_data.(name)(tA:end);
    %                 FM = max(Force);
    %                 figure(j);
    %                 plot(t, Force);
    %                 xlabel('Time [s]');
    %                 ylabel('\it Force(N)');
    %             end
    %             fwrite(Serial, FM, 'uint16');
    %         end
    %         if c ==2
    %             nominal_hit = true;
    %         end
    %     end
end




