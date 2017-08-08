% PARAMETERS
% - SIMULATION
DELTA_T = 0.01; % s
SIMULATION_TIME = 10 % s

% - SIGNAL DISTORTION
% -- Output scale
signal_scale_min = -8.727; % equals -500 deg/s  % in rad/s 
signal_scale_max = 8.727; % equals 500 deg/s    % in rad/s 

% -- Offset
signal_offset = 0.436; % equals 25 deg/s        % in rad/s

% -- Noise
signal_variance = 0.0103;

% -- Cut-Off Frequency

% -- Quantization
signal_resolution_binary_digits = 16; % Bits

% - SAMPLE DATA
t = 0:DELTA_T:SIMULATION_TIME;
inputData = sin(t); % real values, in radians/s


% Calculate derived values
digital_signal_steps = power(2, signal_resolution_binary_digits);
digital_signal_smallest_step = (signal_scale_min*(-1) + signal_scale_max) / digital_signal_steps; % rad/s

% Simulate gyroscope output over time
outputData = zeros(length(inputData), 1); % rad/s
counter = 1;

angle = zeros(length(inputData), 1);

for datapoint = inputData
    input = inputData(counter);
    
    % Apply noise and offset
    gaussian_noise = randn * sqrt(signal_variance);
    output = input + gaussian_noise + signal_offset;
    
    % Apply measurement range clipping
    if output > signal_scale_max
        output = signal_scale_max;
    elseif output < signal_scale_min
        output = signal_scale_min;
    end
    
    % Apply quantization
    output = round(output / digital_signal_smallest_step) * digital_signal_smallest_step;
    
    outputData(counter) = output;
    if counter > 1
        angle(counter) = angle(counter - 1) + output * DELTA_T;
    end
    counter = counter + 1;
end


% Plot input data vs output
figure
plot(t, inputData, t, outputData);
%plot(t, angle);