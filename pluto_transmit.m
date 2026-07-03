%% ================= PC 1: TRANSMITTER =================
clc; clear; close all;

%% 1. PARAMETERS
Fs = 600e3;             
Rb = 60e3;              
Ns = Fs/Rb;             
centerFreq = 915e6;     
rolloff = 0.35;
span = 6;
%% 2. PREAMBLE & IMAGE PROCESSING
warmup_bits = repmat([1; 0], 500, 1); 
warmup_sym = 2*warmup_bits - 1;

preamble_bits = [1 1 1 1 1 0 0 1 1 0 1 0 1]';
preamble_sym = 2*preamble_bits - 1;

% Load and Resize Image
img = imread('C:\Users\nitin\OneDrive\Documents\MATLAB\B2DBy.jpg');
img = imresize(img, [64 64]); 
if size(img,3) == 3
    img = rgb2gray(img);
end
img_vec = img(:);

% Convert image to bits
bits_tx = de2bi(img_vec, 8, 'left-msb');
bits_tx = double(bits_tx.');
bits_tx = bits_tx(:); % These are the ground truth bits

% --- NEW: SAVE BITS FOR BER CALCULATION ---
% Copy this 'tx_bits.mat' file to the Receiver PC's current folder
save('tx_bits.mat', 'bits_tx'); 
fprintf('Ground truth bits saved to tx_bits.mat. Copy this to PC 2.\n');
% ------------------------------------------

data_sym = 2*bits_tx - 1;
tx_symbols = [warmup_sym; preamble_sym; data_sym];

%% 3. TRANSMITTER PIPELINE
txFilter = comm.RaisedCosineTransmitFilter(...
    'RolloffFactor', rolloff, 'FilterSpanInSymbols', span, ...
    'OutputSamplesPerSymbol', Ns);

tx_signal = txFilter(tx_symbols);
tx_signal = tx_signal ./ max(abs(tx_signal)); 
tx_signal = complex(tx_signal); 

%% 4. ADALM-PLUTO HARDWARE SETUP
disp('Initializing ADALM-PLUTO SDR Transmitter...');
tx = sdrtx('Pluto', 'CenterFrequency', centerFreq, ...
    'BasebandSampleRate', Fs, 'Gain', -10); 

disp('Transmitting signal over the air...');
transmitRepeat(tx, tx_signal);