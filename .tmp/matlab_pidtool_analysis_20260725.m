function matlab_pidtool_analysis_20260725()
% Analyze the recorded attitude data with MATLAB identification and PID tools.
% This script does not modify firmware or send commands to the aircraft.

rootDir = fileparts(fileparts(mfilename('fullpath')));
csvPath = fullfile(rootDir, 'flightlog_20260724_200832.csv');
outDir = fullfile(rootDir, '.tmp', 'matlab_pidtool_20260725');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
cachePath = fullfile(outDir, 'workspace.mat');
cachedResults = struct();
if exist(cachePath, 'file')
    cache = load(cachePath, 'results');
    if isfield(cache, 'results')
        cachedResults = cache.results;
    end
end

T = readtable(csvPath, 'VariableNamingRule', 'preserve');
Ts = 0.004;
gapSeconds = diff(double(T.timestamp_us)) * 1e-6;
segmentStarts = [1; find(gapSeconds > 0.1) + 1];
segmentEnds = [segmentStarts(2:end) - 1; height(T)];

sequenceSpan = double(T.sequence(end) - T.sequence(1) + 1);
quality = struct();
quality.rows = height(T);
quality.sequence_span = sequenceSpan;
quality.sequence_coverage = height(T) / sequenceSpan;
quality.missing_records = sequenceSpan - height(T);
quality.large_time_gaps = nnz(gapSeconds > 0.1);
quality.tilt_saturation_fraction = mean( ...
    abs(T.ctrl_tilt_out_rad_0) >= 0.313 | abs(T.ctrl_tilt_out_rad_1) >= 0.313);
quality.sample_period_median_s = median(gapSeconds(gapSeconds < 0.02));

fprintf('Rows=%d, sequence coverage=%.2f%%, missing=%d\n', ...
    quality.rows, 100 * quality.sequence_coverage, quality.missing_records);
fprintf('Large experiment gaps=%d, tilt saturation=%.2f%%\n', ...
    quality.large_time_gaps, 100 * quality.tilt_saturation_fraction);

axesConfig = struct( ...
    'name', {'pitch', 'roll'}, ...
    'servoColumn', {'servo_beta_us', 'servo_alpha_us'}, ...
    'angleColumn', {'pitch_deg', 'roll_deg'}, ...
    'gyroColumn', {'gyro_y_dps', 'gyro_x_dps'}, ...
    'lever_m', {0.145, 0.105}, ...
    'effectiveness', {0.5691933250, 0.5810000000}, ...
    'delay_nom_s', {0.040, 0.060}, ...
    'tau_nom_s', {0.020, 0.040}, ...
    'delay_range_s', {[0.030, 0.050], [0.030, 0.070]}, ...
    'tau_range_s', {[0.020, 0.060], [0.020, 0.060]});

results = struct();
results.generated_at = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
results.source_csv = csvPath;
results.quality = quality;
results.direct_identification_warning = [ ...
    'The direct tfest model uses closed-loop commanded servo position, not measured servo position. ', ...
    'It is diagnostic only and must not be treated as a final free-flight plant.'];

for axisIndex = 1:numel(axesConfig)
    cfg = axesConfig(axisIndex);
    fprintf('\n=== %s ===\n', upper(cfg.name));
    experiments = buildExperiments(T, segmentStarts, segmentEnds, cfg, Ts);
    fprintf('Usable low-saturation experiments=%d\n', numel(experiments));

    if isfield(cachedResults, cfg.name) && ...
            isfield(cachedResults.(cfg.name), 'direct_identification')
        direct = cachedResults.(cfg.name).direct_identification;
        fprintf('Reusing cached direct tfest result: validation fit=%.2f%%\n', ...
            direct.spec.validation_fit_percent);
    else
        direct = fitDirectModels(experiments, Ts, cfg.name);
    end
    physical = tunePhysicalModel(cfg);
    results.(cfg.name) = struct( ...
        'usable_experiments', numel(experiments), ...
        'direct_identification', direct, ...
        'physical_pid_tuning', physical);

    saveAxisPlots(outDir, cfg, physical);
end

jsonPath = fullfile(outDir, 'results.json');
fid = fopen(jsonPath, 'w', 'n', 'UTF-8');
assert(fid >= 0, 'Cannot open JSON output.');
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(results, 'PrettyPrint', true), 'char');
clear cleanup;

writeMarkdown(fullfile(outDir, 'report.md'), results);
save(fullfile(outDir, 'workspace.mat'), 'results', 'quality', 'axesConfig');
fprintf('\nOutputs written to %s\n', outDir);
end

function experiments = buildExperiments(T, starts, ends, cfg, Ts)
experiments = {};
for k = 1:numel(starts)
    rows = starts(k):ends(k);
    block = T(rows, :);
    sat = abs(block.ctrl_tilt_out_rad_0) >= 0.313 | ...
          abs(block.ctrl_tilt_out_rad_1) >= 0.313;
    active = block.imu_valid ~= 0 & block.motor_output_reason == 1;
    if mean(sat) > 0.05 || nnz(active) < 300
        continue;
    end

    block = block(active, :);
    seq = double(block.sequence);
    if numel(seq) < 300 || seq(end) <= seq(1)
        continue;
    end
    seqGrid = (seq(1):seq(end))';

    servo = double(block.(cfg.servoColumn));
    commandRad = -(servo - 1500.0) * pi / 2000.0;
    angleRad = deg2rad(double(block.(cfg.angleColumn)));
    gyroRad = deg2rad(double(block.(cfg.gyroColumn)));

    commandGrid = interp1(seq, commandRad, seqGrid, 'previous', 'extrap');
    angleGrid = interp1(seq, angleRad, seqGrid, 'linear', 'extrap');
    gyroGrid = interp1(seq, gyroRad, seqGrid, 'linear', 'extrap');

    angleRate = gradient(angleGrid, Ts);
    rateSign = sign(corr(angleRate, gyroGrid, 'Rows', 'complete'));
    if rateSign == 0
        rateSign = 1;
    end
    gyroGrid = rateSign * gyroGrid;

    % Remove per-run offsets while preserving the dynamic content.
    commandGrid = detrend(commandGrid, 0);
    angleGrid = detrend(angleGrid, 0);
    gyroGrid = detrend(gyroGrid, 0);
    if std(commandGrid) < deg2rad(0.3)
        continue;
    end

    zAngle = iddata(angleGrid, commandGrid, Ts, ...
        'InputName', 'servo command', 'OutputName', [cfg.name ' angle']);
    zRate = iddata(gyroGrid, commandGrid, Ts, ...
        'InputName', 'servo command', 'OutputName', [cfg.name ' rate']);
    experiments{end + 1} = struct( ...
        'angle', zAngle, 'rate', zRate, 'rate_sign', rateSign, ...
        'samples', numel(seqGrid)); %#ok<AGROW>
end
end

function result = fitDirectModels(experiments, Ts, axisName)
result = struct('status', 'not_enough_data');
if numel(experiments) < 4
    return;
end

trainIndices = 1:2:numel(experiments);
validationIndices = 2:2:numel(experiments);
if isempty(validationIndices)
    return;
end

trainData = experiments{trainIndices(1)}.angle;
for k = trainIndices(2:end)
    trainData = merge(trainData, experiments{k}.angle);
end

options = tfestOptions('Display', 'off');
options.EnforceStability = true;
bestFit = -Inf;
bestModel = [];
bestSpec = struct();

% The delay grid covers the 30-100 ms range seen in the previous grey-box fit.
for poles = 2:4
    for zeros = 0:min(1, poles - 1)
        for delaySamples = 0:5:30
            delaySeconds = delaySamples * Ts;
            try
                model = tfest(trainData, poles, zeros, delaySeconds, options);
                fits = validationFits(experiments, validationIndices, model);
                score = median(fits, 'omitnan');
                if isfinite(score) && score > bestFit
                    bestFit = score;
                    bestModel = model;
                    bestSpec = struct('poles', poles, 'zeros', zeros, ...
                        'delay_s', delaySeconds, 'validation_fit_percent', score, ...
                        'validation_fits_percent', fits);
                end
            catch error
                fprintf('%s tfest skipped: %s\n', axisName, error.message);
            end
        end
    end
end

if isempty(bestModel)
    result = struct('status', 'fit_failed');
    return;
end

modelTf = tf(bestModel);
result = struct();
result.status = 'diagnostic_only';
result.spec = bestSpec;
result.numerator = modelTf.Numerator{1};
result.denominator = modelTf.Denominator{1};
result.input_delay_s = modelTf.InputDelay;
modelPoles = pole(modelTf).';
modelZeros = zero(modelTf).';
result.poles_real = real(modelPoles);
result.poles_imag = imag(modelPoles);
result.zeros_real = real(modelZeros);
result.zeros_imag = imag(modelZeros);
result.stable = isstable(modelTf);
fprintf('Best direct tfest: poles=%d zeros=%d delay=%.0f ms, validation fit=%.2f%%\n', ...
    bestSpec.poles, bestSpec.zeros, 1000 * bestSpec.delay_s, bestFit);
end

function fits = validationFits(experiments, indices, model)
fits = nan(1, numel(indices));
compareOpt = compareOptions('InitialCondition', 'estimate');
for n = 1:numel(indices)
    try
        [~, fit] = compare(experiments{indices(n)}.angle, model, compareOpt);
        fits(n) = fit(1);
    catch
        fits(n) = NaN;
    end
end
end

function result = tunePhysicalModel(cfg)
Jnom = 0.051;
Tnom = 13.016956;
plantGain = cfg.effectiveness * Tnom * cfg.lever_m / Jnom;
Gnom = tf(plantGain, conv([cfg.tau_nom_s, 1], [1, 0, 0]), ...
    'InputDelay', cfg.delay_nom_s);
GnomForAnalysis = pade(Gnom, 4);

phaseTarget = 60;
options = pidtuneOptions('PhaseMargin', phaseTarget);
candidates = struct([]);
upperEdgeIndex = 0;
upperEdgeBandwidth = -Inf;
conservativeIndex = 0;
conservativeBandwidth = -Inf;
bandwidthGrid = 0.5:0.25:5.0;
firmwareGyroFilterTau = 1 / (2 * pi * 80.0);

for bandwidth = bandwidthGrid
    try
        [controller, info] = pidtune(Gnom, 'PDF', bandwidth, options);
        firmwareController = pid(controller.Kp, 0, controller.Kd, ...
            firmwareGyroFilterTau);
        robustness = evaluateUncertainty(firmwareController, cfg);
        closedLoop = feedback(GnomForAnalysis * firmwareController, 1);
        stepData = stepinfo(closedLoop);
        item = struct();
        item.requested_crossover_rad_s = bandwidth;
        item.achieved_crossover_rad_s = info.CrossoverFrequency;
        item.nominal_phase_margin_deg = info.PhaseMargin;
        item.Kp_servo_rad_per_rad = controller.Kp;
        item.Kd_servo_rad_s_per_rad = controller.Kd;
        item.pidtuner_derivative_filter_s = controller.Tf;
        item.firmware_gyro_filter_approx_s = firmwareGyroFilterTau;
        item.firmware_angle_kp_force_N_per_rad = controller.Kp * Tnom;
        item.firmware_rate_kd_Nm_s_per_rad = ...
            -controller.Kd * Tnom * cfg.lever_m;
        item.min_phase_margin_deg = robustness.min_phase_margin_deg;
        item.min_gain_margin_db = robustness.min_gain_margin_db;
        item.all_models_stable = robustness.all_models_stable;
        item.settling_time_s = stepData.SettlingTime;
        item.overshoot_percent = stepData.Overshoot;
        if isempty(candidates)
            candidates = item;
        else
            candidates(end + 1) = item; %#ok<AGROW>
        end

        acceptable = robustness.all_models_stable && ...
            robustness.min_phase_margin_deg >= 50 && ...
            robustness.min_gain_margin_db >= 6;
        conservative = robustness.all_models_stable && ...
            robustness.min_phase_margin_deg >= 60 && ...
            robustness.min_gain_margin_db >= 12;
        if acceptable && bandwidth > upperEdgeBandwidth
            upperEdgeBandwidth = bandwidth;
            upperEdgeIndex = numel(candidates);
        end
        if conservative && bandwidth > conservativeBandwidth
            conservativeBandwidth = bandwidth;
            conservativeIndex = numel(candidates);
        end
    catch error
        fprintf('%s pidtune skipped at %.2f rad/s:\n%s\n', ...
            cfg.name, bandwidth, getReport(error, 'extended', 'hyperlinks', 'off'));
    end
end

result = struct();
result.model = struct( ...
    'inertia_kg_m2', Jnom, 'thrust_nominal_N', Tnom, ...
    'lever_m', cfg.lever_m, 'effectiveness', cfg.effectiveness, ...
    'plant_gain_rad_s2_per_servo_rad', plantGain, ...
    'servo_tau_s', cfg.tau_nom_s, 'delay_s', cfg.delay_nom_s, ...
    'delay_analysis', 'fourth_order_pade');
result.uncertainty = struct( ...
    'inertia_scale', [0.9, 1.1], ...
    'thrust_N', [9.238, 16.849], ...
    'effectiveness_scale', [0.8, 1.2], ...
    'delay_s', cfg.delay_range_s, ...
    'servo_tau_s', cfg.tau_range_s);
result.candidates = candidates;
if upperEdgeIndex > 0
    result.upper_edge_index = upperEdgeIndex;
    result.upper_edge = candidates(upperEdgeIndex);
    fprintf(['Robust upper edge: wc=%.2f rad/s, Kp_force=%.3f N/rad, ', ...
        'Kd_code=%.3f Nm*s/rad, min PM=%.1f deg, min GM=%.1f dB\n'], ...
        candidates(upperEdgeIndex).requested_crossover_rad_s, ...
        candidates(upperEdgeIndex).firmware_angle_kp_force_N_per_rad, ...
        candidates(upperEdgeIndex).firmware_rate_kd_Nm_s_per_rad, ...
        candidates(upperEdgeIndex).min_phase_margin_deg, ...
        candidates(upperEdgeIndex).min_gain_margin_db);
else
    result.upper_edge_index = 0;
    result.upper_edge = struct();
    fprintf('No candidate passed the selected robust margin gates.\n');
end
if conservativeIndex > 0
    result.conservative_index = conservativeIndex;
    result.conservative = candidates(conservativeIndex);
    fprintf(['Conservative analysis point: wc=%.2f rad/s, Kp_force=%.3f N/rad, ', ...
        'Kd_code=%.3f Nm*s/rad, min PM=%.1f deg, min GM=%.1f dB\n'], ...
        candidates(conservativeIndex).requested_crossover_rad_s, ...
        candidates(conservativeIndex).firmware_angle_kp_force_N_per_rad, ...
        candidates(conservativeIndex).firmware_rate_kd_Nm_s_per_rad, ...
        candidates(conservativeIndex).min_phase_margin_deg, ...
        candidates(conservativeIndex).min_gain_margin_db);
else
    result.conservative_index = 0;
    result.conservative = struct();
end
end

function result = evaluateUncertainty(controller, cfg)
Jvalues = 0.051 * [0.9, 1.1];
Tvalues = [9.238, 16.849];
etaValues = cfg.effectiveness * [0.8, 1.2];
delayValues = cfg.delay_range_s;
tauValues = cfg.tau_range_s;

minPm = Inf;
minGmDb = Inf;
allStable = true;
for J = Jvalues
    for thrust = Tvalues
        for eta = etaValues
            for delay = delayValues
                for tau = tauValues
                    gain = eta * thrust * cfg.lever_m / J;
                    plant = tf(gain, conv([tau, 1], [1, 0, 0]), ...
                        'InputDelay', delay);
                    plant = pade(plant, 4);
                    loop = plant * controller;
                    closedLoop = feedback(loop, 1);
                    allStable = allStable && isstable(closedLoop);
                    [gainMargin, phaseMargin] = margin(loop);
                    if isempty(phaseMargin) || isnan(phaseMargin)
                        phaseMargin = -Inf;
                    end
                    if isempty(gainMargin) || isnan(gainMargin)
                        gainMarginDb = -Inf;
                    elseif isinf(gainMargin)
                        gainMarginDb = Inf;
                    else
                        gainMarginDb = 20 * log10(gainMargin);
                    end
                    minPm = min(minPm, phaseMargin);
                    minGmDb = min(minGmDb, gainMarginDb);
                end
            end
        end
    end
end
result = struct('min_phase_margin_deg', minPm, ...
    'min_gain_margin_db', minGmDb, 'all_models_stable', allStable);
end

function saveAxisPlots(outDir, cfg, physical)
if isempty(physical.candidates)
    return;
end
c = physical.candidates;
wc = [c.requested_crossover_rad_s];
pm = [c.min_phase_margin_deg];
gm = [c.min_gain_margin_db];
kp = [c.firmware_angle_kp_force_N_per_rad];
kd = [c.firmware_rate_kd_Nm_s_per_rad];

figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1000, 700]);
tiledlayout(2, 1);
nexttile;
plot(wc, pm, '-o', 'LineWidth', 1.3); hold on;
plot(wc, gm, '-s', 'LineWidth', 1.3);
yline(50, '--'); yline(6, '--'); grid on;
xlabel('Requested crossover (rad/s)');
ylabel('Worst-case margin');
legend('Phase margin (deg)', 'Gain margin (dB)', 'Location', 'best');
title([upper(cfg.name) ' robust margins']);
nexttile;
yyaxis left; plot(wc, kp, '-o', 'LineWidth', 1.3);
ylabel('Firmware angle Kp (N/rad)');
yyaxis right; plot(wc, kd, '-s', 'LineWidth', 1.3);
ylabel('Firmware rate Kd (N m s/rad)');
xlabel('Requested crossover (rad/s)'); grid on;
title([upper(cfg.name) ' gain candidates']);
exportgraphics(gcf, fullfile(outDir, [cfg.name '_pidtune.png']), 'Resolution', 150);
close(gcf);
end

function writeMarkdown(path, results)
fid = fopen(path, 'w', 'n', 'UTF-8');
assert(fid >= 0, 'Cannot open Markdown output.');
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# MATLAB PID Toolbox Analysis\n\n');
fprintf(fid, 'Source: `%s`\n\n', results.source_csv);
q = results.quality;
fprintf(fid, '## Data quality\n\n');
fprintf(fid, '- Rows: %d\n', q.rows);
fprintf(fid, '- Sequence coverage: %.2f%%\n', 100 * q.sequence_coverage);
fprintf(fid, '- Missing records: %d\n', q.missing_records);
fprintf(fid, '- Large experiment gaps: %d\n', q.large_time_gaps);
fprintf(fid, '- Tilt saturation: %.2f%%\n\n', 100 * q.tilt_saturation_fraction);

for axisName = {'pitch', 'roll'}
    name = axisName{1};
    axisResult = results.(name);
    fprintf(fid, '## %s\n\n', upper(name));
    fprintf(fid, '- Usable low-saturation experiments: %d\n', ...
        axisResult.usable_experiments);
    direct = axisResult.direct_identification;
    fprintf(fid, '- Direct tfest status: `%s`\n', direct.status);
    if isfield(direct, 'spec')
        fprintf(fid, '- Direct tfest validation fit: %.2f%%\n', ...
            direct.spec.validation_fit_percent);
        fprintf(fid, '- Direct tfest structure: %d poles, %d zeros, %.0f ms delay\n', ...
            direct.spec.poles, direct.spec.zeros, 1000 * direct.spec.delay_s);
    end
    tune = axisResult.physical_pid_tuning;
    if tune.conservative_index > 0
        r = tune.conservative;
        fprintf(fid, '- Conservative analysis crossover: %.2f rad/s\n', ...
            r.requested_crossover_rad_s);
        fprintf(fid, '- Conservative firmware angle Kp magnitude: %.6f N/rad\n', ...
            r.firmware_angle_kp_force_N_per_rad);
        fprintf(fid, '- Conservative firmware rate Kd: %.6f N m s/rad\n', ...
            r.firmware_rate_kd_Nm_s_per_rad);
        fprintf(fid, '- Worst-case phase margin: %.2f deg\n', ...
            r.min_phase_margin_deg);
        fprintf(fid, '- Worst-case gain margin: %.2f dB\n', ...
            r.min_gain_margin_db);
        fprintf(fid, '- Nominal settling time: %.3f s\n', r.settling_time_s);
        fprintf(fid, '- Nominal overshoot: %.2f%%\n', r.overshoot_percent);
    end
    if tune.upper_edge_index > 0
        edge = tune.upper_edge;
        fprintf(fid, '- Upper-edge crossover: %.2f rad/s\n', ...
            edge.requested_crossover_rad_s);
        fprintf(fid, '- Upper-edge Kp/Kd: %.6f N/rad, %.6f N m s/rad\n', ...
            edge.firmware_angle_kp_force_N_per_rad, ...
            edge.firmware_rate_kd_Nm_s_per_rad);
    end
    if tune.conservative_index == 0 && tune.upper_edge_index == 0
        fprintf(fid, '- No candidate passed the robust margin gates.\n');
    end
    fprintf(fid, '\n');
end

fprintf(fid, '## Interpretation\n\n');
fprintf(fid, ['The direct identified models are diagnostic because the input is a ', ...
    'closed-loop servo command and the log has no measured servo position. ', ...
    'The physical-model PID candidates use the measured inertia, lever arms, ', ...
    'thrust range, and the previous grey-box actuator delay/effectiveness estimates. ', ...
    'They are simulation candidates, not flight-approved gains.\n']);
clear cleanup;
end
