% PARAMETERS
% - SIMULATION
DELTA_T = 0.01; % s

% - SIGNAL DISTORTION
% -- Output scale
signal_scale_min = -4; % rad/s 
signal_scale_max = 4; % rad/s 

% -- Offset
signal_offset = 0.2; % rad/s

% -- Nonlinearity

% -- Noise
signal_variance = 0.005;

% -- Cut-Off Frequency

% -- Quantization
signal_resolution_binary_digits = 7; % Bits

% - SAMPLE DATA
t = 0:DELTA_T:10;
inputData = sin(t);%[0, -0.02, -0.022, -0.019, -0.001, 0.2, 0.72, 0.97]; % real values, in radians/s


% Calculate derived values
digital_signal_steps = power(2, signal_resolution_binary_digits);
digital_signal_smallest_step = (signal_scale_min*(-1) + signal_scale_max) / digital_signal_steps; % rad/s

% Simulate gyroscope output over time
outputData = zeros(length(inputData), 1); % rad/s
counter = 1;

for datapoint = inputData
    input = inputData(counter);
    
    % Apply noise and offset
    gaussian_noise = randn * sqrt(signal_variance);
    output = input + gaussian_noise + signal_offset;
    
    % Apply quantization
    output = round(output / digital_signal_smallest_step) * digital_signal_smallest_step;
    
    outputData(counter) = output;
    counter = counter + 1;
end


% Plot input data vs output
figure
plot(t, inputData, t, outputData);