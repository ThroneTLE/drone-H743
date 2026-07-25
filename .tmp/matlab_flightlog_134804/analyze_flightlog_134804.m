clear; clc;

csvPath = 'D:\stm32hal\drone-H743\flightlog_20260725_134804.csv';
outDir = 'D:\stm32hal\drone-H743\.tmp\matlab_flightlog_134804';
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

opts = detectImportOptions(csvPath, 'FileType', 'text', ...
    'VariableNamingRule', 'preserve');
T = readtable(csvPath, opts);
n = height(T);

timestamp_us = col(T, 'timestamp_us');
sequence = col(T, 'sequence');
t = (timestamp_us - timestamp_us(1)) * 1.0e-6;
dt = diff(t);
dseq = diff(sequence);
segmentId = ones(n, 1);
seg = 1;
for i = 2:n
    if (dt(i - 1) > 0.5) || (dseq(i - 1) > 100)
        seg = seg + 1;
    end
    segmentId(i) = seg;
end
segments = unique(segmentId);

motorReason = string(T.('motor_output_reason_name'));
velLoopActive = col(T, 'vel_loop_active') >= 0.5;
stabilized = motorReason == "stabilized_mix";
throttleOver20 = col(T, 'throttle_over_20') >= 0.5;
active = velLoopActive & stabilized & throttleOver20;

flags = uint32(col(T, 'ctrl_protection_flags'));
tilt0 = col(T, 'ctrl_tilt_out_rad_0');
tilt1 = col(T, 'ctrl_tilt_out_rad_1');
tiltSat = (abs(tilt0) > 0.313) | (abs(tilt1) > 0.313);
motorHighSat = (col(T, 'motor_upper_us') >= 1939) | ...
               (col(T, 'motor_lower_us') >= 1939);

summary = struct();
summary.rows = n;
summary.columns = width(T);
summary.duration_s = t(end) - t(1);
summary.observed_rate_hz = n / max(summary.duration_s, eps);
summary.dt_median_ms = median(dt) * 1000;
summary.dt_p95_ms = prctile(dt, 95) * 1000;
summary.segment_count = numel(segments);
summary.active_rows = nnz(active);
summary.active_pct = 100 * nnz(active) / n;
summary.tilt_saturation_pct = 100 * nnz(tiltSat) / n;
summary.active_tilt_saturation_pct = 100 * nnz(tiltSat & active) / max(nnz(active), 1);
summary.motor_high_saturation_pct = 100 * nnz(motorHighSat) / n;
horizontalScale = col(T, 'ctrl_horizontal_command_scale');
rollDeg = col(T, 'roll_deg');
pitchDeg = col(T, 'pitch_deg');
gyroXDps = col(T, 'gyro_x_dps');
gyroYDps = col(T, 'gyro_y_dps');
summary.horizontal_scale_mean_active = mean(horizontalScale(active), 'omitnan');
summary.roll_rms_deg_active = rms0(rollDeg(active));
summary.pitch_rms_deg_active = rms0(pitchDeg(active));
summary.gyro_x_rms_dps_active = rms0(gyroXDps(active));
summary.gyro_y_rms_dps_active = rms0(gyroYDps(active));

flagNames = ["velocity_invalid", "attitude_tilt", "moment", "thrust"];
flagBits = uint32([1 2 4 8]);
flagRows = table('Size', [numel(flagBits) 4], ...
    'VariableTypes', ["string", "double", "double", "double"], ...
    'VariableNames', ["flag", "count_all", "pct_all", "pct_active"]);
for i = 1:numel(flagBits)
    bitActive = bitand(flags, flagBits(i)) ~= 0;
    flagRows.flag(i) = flagNames(i);
    flagRows.count_all(i) = nnz(bitActive);
    flagRows.pct_all(i) = 100 * nnz(bitActive) / n;
    flagRows.pct_active(i) = 100 * nnz(bitActive & active) / max(nnz(active), 1);
end
writetable(flagRows, fullfile(outDir, 'matlab_protection_flags.csv'));

segRows = table('Size', [numel(segments) 10], ...
    'VariableTypes', repmat("double", 1, 10), ...
    'VariableNames', ["segment", "rows", "duration_s", "active_pct", ...
                      "tilt_sat_pct", "protect_pct", "thrust_protect_pct", ...
                      "roll_rms_deg", "pitch_rms_deg", "gyro_xy_rms_dps"]);
for k = 1:numel(segments)
    idx = segmentId == segments(k);
    tk = t(idx);
    fk = flags(idx);
    gyroXY = hypot(gyroXDps(idx), gyroYDps(idx));
    segRows.segment(k) = segments(k);
    segRows.rows(k) = nnz(idx);
    segRows.duration_s(k) = max(tk) - min(tk);
    segRows.active_pct(k) = 100 * nnz(active & idx) / nnz(idx);
    segRows.tilt_sat_pct(k) = 100 * nnz(tiltSat & idx) / nnz(idx);
    segRows.protect_pct(k) = 100 * nnz(fk ~= 0) / nnz(idx);
    segRows.thrust_protect_pct(k) = 100 * nnz(bitand(fk, uint32(8)) ~= 0) / nnz(idx);
    segRows.roll_rms_deg(k) = rms0(rollDeg(idx));
    segRows.pitch_rms_deg(k) = rms0(pitchDeg(idx));
    segRows.gyro_xy_rms_dps(k) = rms0(gyroXY);
end
writetable(segRows, fullfile(outDir, 'matlab_segments.csv'));

servoRows = servoDelayTable(T, t, segmentId, active);
writetable(servoRows, fullfile(outDir, 'matlab_servo_delay.csv'));

servoFeedbackRows = servoFeedbackStats(T, t, active);
writetable(servoFeedbackRows, fullfile(outDir, 'matlab_servo_feedback_stats.csv'));

freqRows = dominantFrequencyTable(T, t, segmentId, active);
writetable(freqRows, fullfile(outDir, 'matlab_frequency.csv'));

fid = fopen(fullfile(outDir, 'matlab_summary.txt'), 'w');
fprintf(fid, 'CSV: %s\n', csvPath);
fprintf(fid, 'Rows: %d, columns: %d, duration: %.3f s, segments: %d\n', ...
    summary.rows, summary.columns, summary.duration_s, summary.segment_count);
fprintf(fid, 'Observed average rate: %.3f Hz, median dt: %.3f ms, p95 dt: %.3f ms\n', ...
    summary.observed_rate_hz, summary.dt_median_ms, summary.dt_p95_ms);
fprintf(fid, 'Active rows: %d (%.2f%%)\n', summary.active_rows, summary.active_pct);
fprintf(fid, 'Tilt saturation: %.2f%% all, %.2f%% active\n', ...
    summary.tilt_saturation_pct, summary.active_tilt_saturation_pct);
fprintf(fid, 'Motor high saturation: %.2f%% all\n', summary.motor_high_saturation_pct);
fprintf(fid, 'Active horizontal scale mean: %.3f\n', summary.horizontal_scale_mean_active);
fprintf(fid, 'Active roll RMS: %.3f deg, pitch RMS: %.3f deg\n', ...
    summary.roll_rms_deg_active, summary.pitch_rms_deg_active);
fprintf(fid, 'Active gyro RMS: gx %.3f dps, gy %.3f dps\n', ...
    summary.gyro_x_rms_dps_active, summary.gyro_y_rms_dps_active);
fprintf(fid, '\nProtection flags:\n');
for i = 1:height(flagRows)
    fprintf(fid, '  %s: %.2f%% all, %.2f%% active\n', ...
        flagRows.flag(i), flagRows.pct_all(i), flagRows.pct_active(i));
end
fprintf(fid, '\nServo delay estimates:\n');
for i = 1:height(servoRows)
    fprintf(fid, '  %s seg %d: delay %.3f s, corr %.3f, rmse %.2f us, n %d\n', ...
        servoRows.axis(i), servoRows.segment(i), servoRows.delay_s(i), ...
        servoRows.corr(i), servoRows.rmse_us(i), servoRows.samples(i));
end
fprintf(fid, '\nServo feedback freshness:\n');
for i = 1:height(servoFeedbackRows)
    fprintf(fid, ['  %s: valid %.2f%% all, %.2f%% active, active age median %.1f ms, ' ...
        'p95 %.1f ms, sequence update %.2f Hz, median update %.1f ms\n'], ...
        servoFeedbackRows.axis(i), servoFeedbackRows.valid_pct_all(i), ...
        servoFeedbackRows.valid_pct_active(i), servoFeedbackRows.age_median_ms_active(i), ...
        servoFeedbackRows.age_p95_ms_active(i), servoFeedbackRows.sequence_update_hz(i), ...
        servoFeedbackRows.sequence_update_median_ms(i));
end
fprintf(fid, '\nDominant frequencies:\n');
for i = 1:height(freqRows)
    fprintf(fid, '  %s seg %d: %.3f Hz, amp %.3f\n', ...
        freqRows.signal(i), freqRows.segment(i), ...
        freqRows.dominant_hz(i), freqRows.amplitude(i));
end
fclose(fid);

makePlots(T, t, active, flags, tiltSat, outDir, servoRows, freqRows);

disp(fileread(fullfile(outDir, 'matlab_summary.txt')));

function x = col(T, name)
    x = T.(name);
    if iscell(x)
        x = str2double(string(x));
    end
end

function y = rms0(x)
    x = x(isfinite(x));
    if isempty(x)
        y = NaN;
    else
        y = sqrt(mean(x .^ 2));
    end
end

function rows = servoFeedbackStats(T, t, active)
    axes = ["alpha", "beta"]';
    validMask = uint32(col(T, 'servo_feedback_valid_mask'));
    rows = table('Size', [2 8], ...
        'VariableTypes', ["string", "double", "double", "double", "double", "double", "double", "double"], ...
        'VariableNames', ["axis", "valid_pct_all", "valid_pct_active", ...
                          "age_median_ms_active", "age_p95_ms_active", ...
                          "age_max_ms_active", "sequence_update_hz", ...
                          "sequence_update_median_ms"]);
    for a = 1:2
        bit = uint32(a);
        valid = bitand(validMask, bit) ~= 0;
        age = col(T, "servo_" + axes(a) + "_feedback_age_ms");
        seq = col(T, "servo_" + axes(a) + "_feedback_sequence");

        activeValid = active & valid & isfinite(age);
        rows.axis(a) = axes(a);
        rows.valid_pct_all(a) = 100 * nnz(valid) / numel(valid);
        rows.valid_pct_active(a) = 100 * nnz(active & valid) / max(nnz(active), 1);
        rows.age_median_ms_active(a) = median(age(activeValid), 'omitnan');
        rows.age_p95_ms_active(a) = prctile(age(activeValid), 95);
        rows.age_max_ms_active(a) = max(age(activeValid), [], 'omitnan');

        seqValid = valid & isfinite(seq);
        ts = t(seqValid);
        ss = seq(seqValid);
        changed = [true; diff(ss) ~= 0];
        tc = ts(changed);
        if numel(tc) >= 2
            rows.sequence_update_hz(a) = (numel(tc) - 1) / max(tc(end) - tc(1), eps);
            rows.sequence_update_median_ms(a) = median(diff(tc)) * 1000;
        else
            rows.sequence_update_hz(a) = NaN;
            rows.sequence_update_median_ms(a) = NaN;
        end
    end
end

function rows = servoDelayTable(T, t, segmentId, active)
    axes = ["alpha"; "beta"];
    cmdNames = ["servo_alpha_us"; "servo_beta_us"];
    fbNames = ["servo_alpha_feedback_us"; "servo_beta_feedback_us"];
    validBits = [1; 2];
    outAxis = strings(0, 1);
    outSeg = [];
    outDelay = [];
    outCorr = [];
    outSamples = [];
    outCmdStd = [];
    outRmse = [];
    outMeanErr = [];
    segments = unique(segmentId);
    for a = 1:numel(axes)
        cmd = col(T, cmdNames(a));
        fb = col(T, fbNames(a));
        mask = uint32(col(T, 'servo_feedback_valid_mask'));
        valid = bitand(mask, uint32(validBits(a))) ~= 0;
        for s = 1:numel(segments)
            idx = segmentId == segments(s) & active & valid & ...
                  isfinite(cmd) & isfinite(fb);
            if nnz(idx) < 80 || std(cmd(idx), 0, 'omitnan') < 5
                continue;
            end
            [delay, corrBest] = estimateDelay(t(idx), cmd(idx), fb(idx), 100, 0.5);
            err = fb(idx) - cmd(idx);
            outAxis(end + 1, 1) = axes(a); %#ok<AGROW>
            outSeg(end + 1, 1) = segments(s); %#ok<AGROW>
            outDelay(end + 1, 1) = delay; %#ok<AGROW>
            outCorr(end + 1, 1) = corrBest; %#ok<AGROW>
            outSamples(end + 1, 1) = nnz(idx); %#ok<AGROW>
            outCmdStd(end + 1, 1) = std(cmd(idx), 0, 'omitnan'); %#ok<AGROW>
            outRmse(end + 1, 1) = sqrt(mean(err .^ 2, 'omitnan')); %#ok<AGROW>
            outMeanErr(end + 1, 1) = mean(err, 'omitnan'); %#ok<AGROW>
        end
    end
    rows = table(outAxis, outSeg, outDelay, outCorr, outSamples, ...
        outCmdStd, outRmse, outMeanErr, ...
        'VariableNames', ["axis", "segment", "delay_s", "corr", ...
                          "samples", "cmd_std_us", "rmse_us", "mean_error_us"]);
end

function [bestDelay, bestCorr] = estimateDelay(t, cmd, fb, fs, maxLagS)
    t = t(:);
    cmd = cmd(:);
    fb = fb(:);
    [t, order] = sort(t);
    cmd = cmd(order);
    fb = fb(order);
    tu = (ceil(min(t) * fs) / fs : 1 / fs : floor(max(t) * fs) / fs)';
    if numel(tu) < 40
        bestDelay = NaN;
        bestCorr = NaN;
        return;
    end
    cu = interp1(t, cmd, tu, 'linear', NaN);
    fu = interp1(t, fb, tu, 'linear', NaN);
    maxLag = round(maxLagS * fs);
    bestCorr = -Inf;
    bestLag = 0;
    for lag = -maxLag:maxLag
        if lag >= 0
            x = cu(1:end-lag);
            y = fu(1+lag:end);
        else
            x = cu(1-lag:end);
            y = fu(1:end+lag);
        end
        ok = isfinite(x) & isfinite(y);
        if nnz(ok) < 30
            continue;
        end
        x = x(ok) - mean(x(ok));
        y = y(ok) - mean(y(ok));
        den = sqrt(sum(x .^ 2) * sum(y .^ 2));
        if den <= eps
            continue;
        end
        c = sum(x .* y) / den;
        if c > bestCorr
            bestCorr = c;
            bestLag = lag;
        end
    end
    bestDelay = bestLag / fs;
end

function rows = dominantFrequencyTable(T, t, segmentId, active)
    sigNames = ["roll_deg"; "pitch_deg"; "gyro_x_dps"; "gyro_y_dps"; ...
                "ctrl_tilt_out_rad_0"; "ctrl_tilt_out_rad_1"];
    outSignal = strings(0, 1);
    outSeg = [];
    outHz = [];
    outAmp = [];
    segments = unique(segmentId);
    for s = 1:numel(segments)
        idxSeg = segmentId == segments(s) & active;
        if nnz(idxSeg) < 150
            continue;
        end
        for k = 1:numel(sigNames)
            x = col(T, sigNames(k));
            [hz, amp] = dominantHz(t(idxSeg), x(idxSeg), 100);
            if ~isfinite(hz)
                continue;
            end
            outSignal(end + 1, 1) = sigNames(k); %#ok<AGROW>
            outSeg(end + 1, 1) = segments(s); %#ok<AGROW>
            outHz(end + 1, 1) = hz; %#ok<AGROW>
            outAmp(end + 1, 1) = amp; %#ok<AGROW>
        end
    end
    rows = table(outSignal, outSeg, outHz, outAmp, ...
        'VariableNames', ["signal", "segment", "dominant_hz", "amplitude"]);
end

function [hz, amp] = dominantHz(t, x, fs)
    t = t(:);
    x = x(:);
    ok = isfinite(t) & isfinite(x);
    t = t(ok);
    x = x(ok);
    if numel(t) < 80 || max(t) <= min(t)
        hz = NaN;
        amp = NaN;
        return;
    end
    [t, order] = sort(t);
    x = x(order);
    tu = (ceil(min(t) * fs) / fs : 1 / fs : floor(max(t) * fs) / fs)';
    if numel(tu) < 80
        hz = NaN;
        amp = NaN;
        return;
    end
    xu = interp1(t, x, tu, 'linear', NaN);
    ok = isfinite(xu);
    xu = xu(ok);
    if numel(xu) < 80
        hz = NaN;
        amp = NaN;
        return;
    end
    xu = xu - mean(xu);
    nfft = 2 ^ nextpow2(numel(xu));
    X = abs(fft(xu, nfft)) / numel(xu);
    f = (0:nfft-1)' * fs / nfft;
    keep = f >= 0.3 & f <= 20;
    if ~any(keep)
        hz = NaN;
        amp = NaN;
        return;
    end
    [amp, rel] = max(X(keep));
    fkeep = f(keep);
    hz = fkeep(rel);
end

function makePlots(T, t, active, flags, tiltSat, outDir, servoRows, freqRows)
    f = figure('Visible', 'off', 'Position', [100 100 1500 1000]);
    tiledlayout(5, 1, 'TileSpacing', 'compact');
    nexttile;
    plot(t, col(T, 'roll_deg'), 'b'); hold on;
    plot(t, col(T, 'pitch_deg'), 'r');
    ylabel('deg'); legend('roll', 'pitch'); grid on;
    title('Attitude');
    nexttile;
    plot(t, col(T, 'vel_est_m_s_0'), 'b'); hold on;
    plot(t, col(T, 'vel_est_m_s_1'), 'r');
    plot(t, col(T, 'vel_ref_m_s_0'), 'b--');
    plot(t, col(T, 'vel_ref_m_s_1'), 'r--');
    ylabel('m/s'); legend('vx', 'vy', 'vx ref', 'vy ref'); grid on;
    title('Velocity loop');
    nexttile;
    plot(t, col(T, 'servo_alpha_us'), 'b'); hold on;
    plot(t, col(T, 'servo_alpha_feedback_us'), 'c');
    plot(t, col(T, 'servo_beta_us'), 'r');
    plot(t, col(T, 'servo_beta_feedback_us'), 'm');
    ylabel('us'); legend('alpha cmd', 'alpha fb', 'beta cmd', 'beta fb'); grid on;
    title('Servo command and feedback');
    nexttile;
    plot(t, col(T, 'ctrl_horizontal_command_scale'), 'k'); hold on;
    plot(t, double(flags ~= 0), 'r');
    plot(t, double(tiltSat), 'b');
    ylabel('logic'); legend('h scale', 'any protect', 'tilt sat'); grid on;
    title('Protection and saturation');
    nexttile;
    plot(t, double(active), 'g');
    ylabel('active'); xlabel('time s'); grid on;
    title('Velocity-loop active mask');
    exportgraphics(f, fullfile(outDir, 'matlab_overview.png'), 'Resolution', 150);
    close(f);

    if ~isempty(servoRows)
        f = figure('Visible', 'off', 'Position', [100 100 1000 600]);
        tiledlayout(2, 1, 'TileSpacing', 'compact');
        for a = 1:2
            axName = ["alpha", "beta"];
            nexttile;
            rows = servoRows.axis == axName(a);
            if any(rows)
                bar(categorical(string(servoRows.segment(rows))), servoRows.delay_s(rows));
                ylabel('delay s');
                title(axName(a) + " command to feedback delay");
                grid on;
            end
        end
        exportgraphics(f, fullfile(outDir, 'matlab_servo_delay.png'), 'Resolution', 150);
        close(f);
    end

    if ~isempty(freqRows)
        f = figure('Visible', 'off', 'Position', [100 100 1200 700]);
        signals = unique(freqRows.signal);
        hold on;
        for i = 1:numel(signals)
            rows = freqRows.signal == signals(i);
            scatter(freqRows.segment(rows), freqRows.dominant_hz(rows), 36, 'filled');
        end
        xlabel('segment');
        ylabel('dominant Hz');
        legend(signals, 'Interpreter', 'none', 'Location', 'bestoutside');
        grid on;
        title('Dominant active-segment oscillation frequencies');
        exportgraphics(f, fullfile(outDir, 'matlab_frequency.png'), 'Resolution', 150);
        close(f);
    end
end
