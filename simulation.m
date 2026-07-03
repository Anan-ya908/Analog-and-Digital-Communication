clc; clear; close all;

%% ================= PARAMETERS =================
Rb = 1e3;
Tb = 1/Rb;

Fs = 1e5;                 % Sampling frequency
Ns = round(Fs*Tb);        % Samples per bit

snr_values = 0:5:50;     % Proper SNR range

%% ================= IMAGE =================
img = imread('C:\Users\nitin\OneDrive\Documents\MATLAB\B2DBy.jpg');
img = imresize(img,[64 64]);

if size(img,3)==3
    img = rgb2gray(img);
end

img_vec = img(:);

%% ================= BINARY =================
bits = de2bi(img_vec,8,'left-msb');
bits = bits.';
bits = bits(:);

bits = double(bits);   % 🔥 IMPORTANT FIX

Nb = length(bits);

%% ================= BPSK =================
symbols = 2*bits - 1;

%% ================= RRC FILTER =================
rolloff = 0.35;
span = 6;

rrc = rcosdesign(rolloff, span, Ns);

%% ================= TRANSMITTER =================
tx_signal = upfirdn(symbols, rrc, Ns, 1);

% Normalize once (only here)
tx_signal = tx_signal ./ max(abs(tx_signal));

%% ================= DISPLAY =================
figure;
subplot(3,4,1);
imshow(img);
title('Original');

plot_idx = 2;

%% ================= SNR LOOP =================
for snr = snr_values

    %% ===== AWGN CHANNEL =====
    rx_signal = awgn(tx_signal, snr, 'measured');

    %% ===== MATCHED FILTER =====
    rx_filt = conv(rx_signal, rrc, 'same');

    %% ===== REMOVE FILTER DELAY =====
    delay = span * Ns / 2;
    rx_filt = rx_filt(delay+1:end);

    %% ===== ADD TIMING OFFSET (REALISTIC) =====
    offset = round(Ns/3);

    %% ===== SAMPLING =====
    sample_idx = offset:Ns:Nb*Ns;
    sample_idx = sample_idx(sample_idx <= length(rx_filt));

    rx_samples = rx_filt(sample_idx);

    %% ===== DETECTION =====
    bits_rx = real(rx_samples) > 0;
    bits_rx = bits_rx(:);

    %% ===== LENGTH FIX =====
    if length(bits_rx) < Nb
        bits_rx(end+1:Nb) = 0;
    elseif length(bits_rx) > Nb
        bits_rx = bits_rx(1:Nb);
    end

    %% ===== BER =====
    BER = mean(bits ~= bits_rx);

    %% ===== IMAGE RECONSTRUCTION =====
    bits_rx_matrix = reshape(bits_rx,8,[]).';
    img_rx_vec = bi2de(bits_rx_matrix,'left-msb');

    img_rx = reshape(uint8(img_rx_vec), size(img));

    %% ===== DISPLAY =====
    subplot(3,4,plot_idx);
    imshow(img_rx);
    title(['SNR=', num2str(snr), ', BER=', num2str(BER,3)]);

    plot_idx = plot_idx + 1;
end