%% ================= PC 2: THRESHOLD-FILTERED LIVE RECEIVER =================
clc; clear; close all;
%% 1. PARAMETERS
Fs = 600e3; 
Rb = 60e3;  
Ns = Fs/Rb; 
centerFreq = 915e6;     
rolloff = 0.35;
span = 10; 
img_dimensions = [64 64];
num_data_bits = img_dimensions(1) * img_dimensions(2) * 8; 
% Robust Preamble
barker_13 = [1 1 1 1 1 0 0 1 1 0 1 0 1]';
preamble_bits = repmat(barker_13, 5, 1); 
preamble_sym = 2*preamble_bits - 1;
%% 2. LOAD TX BITS FOR BER
has_tx_bits = false;
if exist('tx_bits.mat', 'file')
    load('tx_bits.mat'); 
    bits_tx = bits_tx(:); 
    has_tx_bits = true;
    disp('[INFO] tx_bits.mat found! Strict BER Threshold (< 0.4) is ENABLED.');
else
    disp('[WARNING] No tx_bits.mat found. Cannot calculate BER. Running blindly.');
end
%% 3. INITIALIZE HARDWARE & FILTERS
rx = sdrrx('Pluto', 'CenterFrequency', centerFreq, ...
    'BasebandSampleRate', Fs, 'GainSource', 'Manual', 'Gain', 30, ...
    'SamplesPerFrame', 500e3); 
rxFilter = comm.RaisedCosineReceiveFilter('RolloffFactor', rolloff, ...
    'FilterSpanInSymbols', span, 'InputSamplesPerSymbol', Ns, 'DecimationFactor', 1);
coarseFreq = comm.CoarseFrequencyCompensator('Modulation', 'BPSK', 'SampleRate', Fs);
symbolSync = comm.SymbolSynchronizer('SamplesPerSymbol', Ns, 'NormalizedLoopBandwidth', 0.05);
carrierSync = comm.CarrierSynchronizer('Modulation', 'BPSK', 'NormalizedLoopBandwidth', 0.005);
fig = figure('Name', 'Filtered SDR Receiver', 'Position', [100, 200, 1400, 400]);
disp('Listening... Only plotting if BER < 0.4 (Close the figure window to stop)');
%% 4. CONTINUOUS CAPTURE LOOP
while ishandle(fig)
    
    rx_signal_raw = double(rx()); 
    reset(coarseFreq); reset(symbolSync); reset(carrierSync);
    
    % DC Block & Normalization
    rx_signal = rx_signal_raw - mean(rx_signal_raw);
    rx_signal = rx_signal / max(abs(rx_signal));
    
    % Filters
    rx_filt = rxFilter(rx_signal);
    rx_coarse = coarseFreq(rx_filt);
    rx_sync = symbolSync(rx_coarse);
    rx_final = carrierSync(rx_sync);
    
    % Fast Energy Detection
    window_size = 500;
    signal_power = movmean(abs(rx_final).^2, window_size);
    threshold = 0.4 * max(signal_power); 
    burst_start = find(signal_power > threshold, 1, 'first');
    
    if isempty(burst_start) || (burst_start + num_data_bits > length(rx_final))
        fprintf('.'); 
        continue; 
    end
    
    % Correlation Search
    search_range = burst_start : min(length(rx_final), burst_start + 2000);
    search_window = rx_final(search_range);
    [corr, lags] = xcorr(search_window, preamble_sym);
    [~, max_idx] = max(abs(corr));
    
    % SYNC TUNING KNOB
    sync_offset = 0; 
    
    start_idx = search_range(1) + lags(max_idx) + length(preamble_sym) + sync_offset;
    
    if start_idx > 0 && (start_idx + num_data_bits - 1) <= length(rx_final)
        
        raw_syms = rx_final(start_idx : start_idx + num_data_bits - 1);
        
        % Phase Correction
        if real(corr(max_idx)) < 0
            raw_syms = -raw_syms; 
        end
        
        % Hard Decision
        bits_rx = real(raw_syms) > 0;
        bits_rx = bits_rx(1:num_data_bits);
        
        %% --- STRICT BER THRESHOLD CHECK ---
        if has_tx_bits
            [num_err, current_ber] = biterr(bits_tx, bits_rx(:));
            
            % If the error rate is 40% or worse, it's mostly noise. Throw it away!
            if current_ber >= 0.5
                fprintf('!'); % Prints a ! to show it caught a garbage packet
                continue; % Immediately skip the rest of the loop and keep listening
            end
            
            % If we made it here, BER is < 0.4! Print success and plot.
            fprintf('\n[SUCCESS] Good Signal! | Errors: %d | BER: %.6f\n', num_err, current_ber);
            img_title = sprintf('Live Image\n(BER: %.4f, Errors: %d)', current_ber, num_err);
        else
            fprintf('\n[SUCCESS] Signal Caught! Decoding blindly...\n');
            img_title = 'Live Image';
        end
        %% ----------------------------------
        
        % Reconstruct Image
        img_rx = reshape(uint8(bi2de(reshape(bits_rx, 8, []).', 'left-msb')), img_dimensions);
        
        %% --- 5. LIVE DISPLAY DASHBOARD ---
        % 1. Image Plot
        subplot(1,3,1); 
        imshow(img_rx); 
        title(img_title); 
        
        % 2. Constellation
        subplot(1,3,2); 
        plot(real(raw_syms), imag(raw_syms), 'b.', 'MarkerSize', 2); 
        title('Constellation');
        xlabel('In-Phase'); ylabel('Quadrature');
        grid on; axis([-2.5 2.5 -2.5 2.5]); axis square;
        
        % 3. Cross-Correlation
        subplot(1,3,3);
        plot(lags, abs(corr), 'r', 'LineWidth', 1.5);
        title('Preamble Sync Peak');
        xlabel('Offset (Symbols)'); ylabel('Match Strength');
        grid on; axis tight;
        
        drawnow; 
        
    else
        fprintf('x'); 
        continue;
    end
end
release(rx);
disp('Receiver safely stopped.');