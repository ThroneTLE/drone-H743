function matlab_closed_loop_id_20260725()
% Closed-loop attitude identification and controller-in-the-loop tuning.
% Read-only: this script never writes firmware or sends hardware commands.

rootDir = fileparts(fileparts(mfilename('fullpath')));
csvPath = fullfile(rootDir, 'flightlog_20260724_200832.csv');
metaPath = fullfile(rootDir, 'flightlog_20260724_200832_meta.json');
outDir = fullfile(rootDir, '.tmp', 'matlab_force_frame_corrected_20260725');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
reusePath = fullfile(outDir, 'results.json');
priorResults = struct();
if exist(reusePath, 'file')
    priorResults = jsondecode(fileread(reusePath));
end

fprintf('Loading flight log...\n');
T = readtable(csvPath, 'VariableNamingRule', 'preserve');
meta = jsondecode(fileread(metaPath));
Ts = 1 / str2double(meta.begin.log_rate);
params = attachSectorParams(T, meta);

fprintf('Reconstructing the controller used during capture...\n');
reconstruction = reconstructController(T, params);
fprintf(['Best force reconstruction: roll/pitch frame signs=%+.0f/%+.0f, ' ...
         'P-force sign=%+.0f, ' ...
         'Fx RMSE=%.5f N, Fy RMSE=%.5f N\n'], ...
        reconstruction.best.roll_frame_sign, ...
        reconstruction.best.pitch_frame_sign, ...
        reconstruction.best.attitude_force_sign, ...
        reconstruction.best.force_x_rmse_n, ...
        reconstruction.best.force_y_rmse_n);
fprintf('Tilt output reconstruction RMSE: pitch=%.6f rad, roll=%.6f rad\n', ...
        reconstruction.tilt.pitch_rmse_rad, reconstruction.tilt.roll_rmse_rad);

axesCfg = struct( ...
    'name', {'pitch', 'roll'}, ...
    'angle_column', {'pitch_deg', 'roll_deg'}, ...
    'gyro_column', {'gyro_y_dps', 'gyro_x_dps'}, ...
    'tilt_column', {'ctrl_tilt_out_rad_0', 'ctrl_tilt_out_rad_1'}, ...
    'rate_sign', {1.0, -1.0}, ...
    'plant_sign', {1.0, -1.0}, ...
    'lever_m', {0.145, 0.105}, ...
    'previous_effectiveness', {0.5691933250, 0.5809669373}, ...
    'previous_tau_s', {0.02, 0.02}, ...
    'previous_delay_s', {0.04, 0.06}, ...
    'previous_simple_kp_force', {0.4179648853, 0.3999962182}, ...
    'previous_simple_kd', {-0.1756773795, -0.1506578310});

segments = buildSegments(T, params, Ts);
fprintf('Continuous experiment segments=%d\n', numel(segments));

results = struct();
results.generated_at = char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss Z'));
results.source_csv = csvPath;
results.method = [ ...
    'Closed-loop linear grey-box prediction-error identification with an ', ...
    'estimated innovation disturbance model, fixed actuator delay family, ', ...
    'whole-experiment two-fold validation, explicit historical force-frame ', ...
    'sign reconstruction, and corrected-controller robust tuning.'];
results.data_quality = dataQuality(T, segments);
results.controller_reconstruction = reconstruction;
results.force_frame_interference = quantifyFrameSignInterference(T, params, reconstruction);
results.fixed_physics = struct( ...
    'mass_kg', 1.367, 'gravity_m_s2', 9.81, ...
    'inertia_kg_m2', 0.051, ...
    'pitch_lever_m', 0.145, 'roll_lever_m', 0.105, ...
    'tilt_limit_rad', deg2rad(18), ...
    'motor_tau_s', 0.176624, 'gyro_filter_hz', 80);

for axisIndex = 1:numel(axesCfg)
    cfg = axesCfg(axisIndex);
    fprintf('\n=== %s CLOSED-LOOP IDENTIFICATION ===\n', upper(cfg.name));
    experiments = buildAxisExperiments(segments, cfg, Ts);
    feedback = detectFeedback(experiments, Ts);
    fprintf('Usable low-saturation experiments=%d, feedback detected=%d/%d\n', ...
            numel(experiments), feedback.detected_count, feedback.tested_count);

    if isfield(priorResults, cfg.name) && ...
            isfield(priorResults.(cfg.name), 'closed_loop_identification')
        identified = priorResults.(cfg.name).closed_loop_identification;
        fprintf('Reusing previous closed-loop delay-family result.\n');
    else
        identified = identifyDelayFamily(experiments, cfg);
    end
    fprintf(['Best fixed delay=%.0f ms, validation angle/rate fit=%.2f/%.2f%%, ' ...
             'tau=%.1f ms, effectiveness=%+.3f\n'], ...
            1000 * identified.best_delay_s, ...
            identified.validation_angle_fit_percent, ...
            identified.validation_rate_fit_percent, ...
            1000 * identified.final_parameters.servo_tau_s, ...
            identified.final_parameters.effectiveness);

    tuning = tuneControllerInLoop(cfg, identified);
    results.(cfg.name) = struct( ...
        'feedback_check', feedback, ...
        'closed_loop_identification', identified, ...
        'controller_in_loop_tuning', tuning, ...
        'previous_simple_pidtuner', struct( ...
            'angle_kp_force_n_per_rad', cfg.previous_simple_kp_force, ...
            'rate_kd_n_m_s_per_rad', cfg.previous_simple_kd, ...
            'warning', ['The previous result omitted the force-frame geometric ', ...
                        'feedback and is comparison-only.']));

    if tuning.candidate_found || ...
            (isfield(tuning, 'physical_candidate_found') && tuning.physical_candidate_found)
        c = tuning.recommended;
        fprintf(['Corrected-controller physical candidate: Kp_code=%+.3f N/rad, ' ...
                 'Kd_code=%+.3f N*m*s/rad, min PM=%.1f deg, min GM=%.1f dB\n'], ...
                c.kp_code_n_per_rad, c.kd_code_n_m_s_per_rad, ...
                c.min_phase_margin_deg, c.min_gain_margin_db);
    else
        fprintf('No candidate passed all robust constraints for this axis.\n');
    end
end

savePlots(outDir, results);
jsonPath = fullfile(outDir, 'results.json');
fid = fopen(jsonPath, 'w', 'n', 'UTF-8');
assert(fid >= 0, 'Cannot open JSON output.');
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(results, 'PrettyPrint', true), 'char');
clear cleanup;
writeReport(fullfile(outDir, 'report.md'), results);
save(fullfile(outDir, 'workspace.mat'), 'results', 'axesCfg');
fprintf('\nOutputs written to %s\n', outDir);
end

function quality = dataQuality(T, segments)
sequenceSpan = double(T.sequence(end) - T.sequence(1) + 1);
dt = diff(double(T.timestamp_us)) * 1e-6;
quality = struct( ...
    'rows', height(T), ...
    'sequence_span', sequenceSpan, ...
    'sequence_coverage_fraction', height(T) / sequenceSpan, ...
    'missing_records', sequenceSpan - height(T), ...
    'large_time_gaps', nnz(dt > 0.1), ...
    'continuous_segments', numel(segments), ...
    'tilt_saturation_fraction', mean( ...
        abs(T.ctrl_tilt_out_rad_0) >= 0.313 | ...
        abs(T.ctrl_tilt_out_rad_1) >= 0.313), ...
    'median_short_sample_period_s', median(dt(dt < 0.02)));
end

function params = attachSectorParams(T, meta)
fields = {'mass_kg', 'gravity_m_s2', 'tilt_lever_arm_m', ...
          'roll_angle_kp', 'pitch_angle_kp', ...
          'roll_rate_kd', 'pitch_rate_kd', 'tilt_limit_rad'};
lookup = containers.Map('KeyType', 'char', 'ValueType', 'any');
for k = 1:numel(meta.sectors)
    sector = meta.sectors(k);
    key = sprintf('%u_%u', sector.sector_seq, sector.sector_index);
    lookup(key) = sector.params;
end

for f = 1:numel(fields)
    params.(fields{f}) = nan(height(T), 1);
end
for row = 1:height(T)
    key = sprintf('%u_%u', T.sector_seq(row), T.sector_index(row));
    if ~isKey(lookup, key)
        continue;
    end
    item = lookup(key);
    for f = 1:numel(fields)
        name = fields{f};
        if isfield(item, name)
            params.(name)(row) = double(item.(name));
        end
    end
end
end

function reconstruction = reconstructController(T, params)
valid = T.imu_valid ~= 0 & T.motor_output_reason == 1 & ...
        isfinite(params.mass_kg) & isfinite(params.pitch_angle_kp);
rows = find(valid);
rollFrameSigns = [-1, 1];
pitchFrameSigns = [-1, 1];
attitudeForceSigns = [-1, 1];
candidates = struct([]);

for rollSign = rollFrameSigns
    for pitchSign = pitchFrameSigns
        for forceSign = attitudeForceSigns
            [fx, fy, fz] = reconstructForceRows( ...
                T, params, rows, rollSign, pitchSign, forceSign);
            item = struct( ...
                'roll_frame_sign', rollSign, ...
                'pitch_frame_sign', pitchSign, ...
                'attitude_force_sign', forceSign, ...
                'force_x_rmse_n', rms(fx - double(T.ctrl_force_cmd_n_0(rows))), ...
                'force_y_rmse_n', rms(fy - double(T.ctrl_force_cmd_n_1(rows))), ...
                'force_z_rmse_n', rms(fz - double(T.ctrl_force_cmd_n_2(rows))));
            item.score_n = sqrt(item.force_x_rmse_n^2 + item.force_y_rmse_n^2 + ...
                                item.force_z_rmse_n^2);
            if isempty(candidates)
                candidates = item;
            else
                candidates(end + 1) = item; %#ok<AGROW>
            end
        end
    end
end
[~, bestIndex] = min([candidates.score_n]);
best = candidates(bestIndex);

fx = double(T.ctrl_force_cmd_n_0(rows));
fy = double(T.ctrl_force_cmd_n_1(rows));
fz = double(T.ctrl_force_cmd_n_2(rows));
alphaFf = atan2(fx, fz);
legacyLever = params.tilt_lever_arm_m(rows);
rateScale = max(fz .* legacyLever, 1e-6);
pitchD = params.pitch_rate_kd(rows) .* deg2rad(double(T.gyro_y_dps(rows))) ./ rateScale;
alpha = clamp(alphaFf + pitchD, ...
              -params.tilt_limit_rad(rows), params.tilt_limit_rad(rows));
betaFf = -atan2(fy .* cos(alpha), fz);
rollD = params.roll_rate_kd(rows) .* deg2rad(double(T.gyro_x_dps(rows))) ./ rateScale;
beta = clamp(betaFf + rollD, ...
             -params.tilt_limit_rad(rows), params.tilt_limit_rad(rows));

reconstruction = struct();
reconstruction.tested_rows = numel(rows);
reconstruction.candidates = candidates;
reconstruction.best = best;
reconstruction.tilt = struct( ...
    'pitch_ff_rmse_rad', rms(alphaFf - double(T.ctrl_tilt_ff_rad_0(rows))), ...
    'roll_ff_rmse_rad', rms(betaFf - double(T.ctrl_tilt_ff_rad_1(rows))), ...
    'pitch_rate_d_rmse_rad', rms(pitchD - double(T.ctrl_tilt_rate_d_rad_0(rows))), ...
    'roll_rate_d_rmse_rad', rms(rollD - double(T.ctrl_tilt_rate_d_rad_1(rows))), ...
    'pitch_rmse_rad', rms(alpha - double(T.ctrl_tilt_out_rad_0(rows))), ...
    'roll_rmse_rad', rms(beta - double(T.ctrl_tilt_out_rad_1(rows))));
reconstruction.capture_model = struct( ...
    'roll_frame_sign', best.roll_frame_sign, ...
    'pitch_frame_sign', best.pitch_frame_sign, ...
    'attitude_feedback_location', 'force_vector', ...
    'attitude_force_sign', best.attitude_force_sign, ...
    'shared_rate_lever_m', median(legacyLever, 'omitnan'));
end

function [fx, fy, fz] = reconstructForceRows( ...
    T, params, rows, rollSign, pitchSign, forceSign)
fx = zeros(numel(rows), 1);
fy = zeros(numel(rows), 1);
fz = zeros(numel(rows), 1);
for n = 1:numel(rows)
    i = rows(n);
    roll = rollSign * deg2rad(double(T.roll_deg(i)));
    pitch = pitchSign * deg2rad(double(T.pitch_deg(i)));
    yaw = deg2rad(double(T.yaw_deg(i)));
    accelLocal = [double(T.ctrl_accel_out_m_s2_0(i)); ...
                  double(T.ctrl_accel_out_m_s2_1(i)); ...
                  params.gravity_m_s2(i) - double(T.ctrl_accel_out_m_s2_2(i))];
    velLocal = [double(T.vel_est_m_s_0(i)); ...
                double(T.vel_est_m_s_1(i)); ...
                double(T.vel_est_m_s_2(i))];
    omega = deg2rad([double(T.gyro_x_dps(i)); ...
                     double(T.gyro_y_dps(i)); ...
                     double(T.gyro_z_dps(i))]);
    R = localDownToBodyMatrix(roll, pitch, yaw);
    accelBody = R * accelLocal;
    velBody = R * velLocal;
    forceBody = params.mass_kg(i) * (accelBody - cross(omega, velBody));
    forceBody(1) = forceBody(1) + forceSign * ...
        params.pitch_angle_kp(i) * deg2rad(double(T.pitch_deg(i)));
    forceBody(2) = forceBody(2) + forceSign * ...
        params.roll_angle_kp(i) * deg2rad(double(T.roll_deg(i)));
    forceBody(3) = max(forceBody(3), 1e-4);
    fx(n) = forceBody(1);
    fy(n) = forceBody(2);
    fz(n) = forceBody(3);
end
end

function result = quantifyFrameSignInterference(T, params, reconstruction)
valid = T.imu_valid ~= 0 & T.motor_output_reason == 1 & ...
        isfinite(params.mass_kg) & isfinite(params.pitch_angle_kp);
rows = find(valid);
best = reconstruction.best;
[captureX, captureY, captureZ] = reconstructForceRows( ...
    T, params, rows, best.roll_frame_sign, best.pitch_frame_sign, ...
    best.attitude_force_sign);
[correctedX, correctedY, correctedZ] = reconstructForceRows( ...
    T, params, rows, -1, 1, best.attitude_force_sign);

pitchAngle = deg2rad(double(T.pitch_deg(rows)));
rollAngle = deg2rad(double(T.roll_deg(rows)));
pitchP = params.pitch_angle_kp(rows) .* pitchAngle;
rollP = params.roll_angle_kp(rows) .* rollAngle;
forceDeltaX = correctedX - captureX;
forceDeltaY = correctedY - captureY;

legacyLever = params.tilt_lever_arm_m(rows);
rateScale = max(double(T.ctrl_total_force_n(rows)) .* legacyLever, 1e-6);
pitchD = params.pitch_rate_kd(rows) .* ...
    deg2rad(double(T.gyro_y_dps(rows))) ./ rateScale;
rollD = params.roll_rate_kd(rows) .* ...
    deg2rad(double(T.gyro_x_dps(rows))) ./ rateScale;
captureAlpha = clamp(atan2(captureX, captureZ) + pitchD, ...
    -params.tilt_limit_rad(rows), params.tilt_limit_rad(rows));
correctedAlpha = clamp(atan2(correctedX, correctedZ) + pitchD, ...
    -params.tilt_limit_rad(rows), params.tilt_limit_rad(rows));
captureBeta = clamp(-atan2(captureY .* cos(captureAlpha), captureZ) + rollD, ...
    -params.tilt_limit_rad(rows), params.tilt_limit_rad(rows));
correctedBeta = clamp(-atan2(correctedY .* cos(correctedAlpha), correctedZ) + rollD, ...
    -params.tilt_limit_rad(rows), params.tilt_limit_rad(rows));

result = struct( ...
    'rows', numel(rows), ...
    'capture_roll_frame_sign', best.roll_frame_sign, ...
    'capture_pitch_frame_sign', best.pitch_frame_sign, ...
    'corrected_roll_frame_sign', -1, ...
    'corrected_pitch_frame_sign', 1, ...
    'force_delta_x_rms_n', rms(forceDeltaX), ...
    'force_delta_y_rms_n', rms(forceDeltaY), ...
    'force_delta_x_p95_abs_n', prctile(abs(forceDeltaX), 95), ...
    'force_delta_y_p95_abs_n', prctile(abs(forceDeltaY), 95), ...
    'pitch_p_term_rms_n', rms(pitchP), ...
    'roll_p_term_rms_n', rms(rollP), ...
    'force_delta_to_p_rms_ratio_pitch', rms(forceDeltaX) / max(rms(pitchP), 1e-6), ...
    'force_delta_to_p_rms_ratio_roll', rms(forceDeltaY) / max(rms(rollP), 1e-6), ...
    'tilt_delta_pitch_rms_deg', rad2deg(rms(correctedAlpha - captureAlpha)), ...
    'tilt_delta_roll_rms_deg', rad2deg(rms(correctedBeta - captureBeta)), ...
    'tilt_delta_pitch_p95_abs_deg', rad2deg(prctile(abs(correctedAlpha - captureAlpha), 95)), ...
    'tilt_delta_roll_p95_abs_deg', rad2deg(prctile(abs(correctedBeta - captureBeta), 95)), ...
    'capture_saturation_fraction', mean( ...
        abs(captureAlpha) >= deg2rad(17.9) | abs(captureBeta) >= deg2rad(17.9)), ...
    'corrected_counterfactual_saturation_fraction', mean( ...
        abs(correctedAlpha) >= deg2rad(17.9) | abs(correctedBeta) >= deg2rad(17.9)));
end

function R = localDownToBodyMatrix(phi, theta, psi)
cphi = cos(phi); sphi = sin(phi);
ctheta = cos(theta); stheta = sin(theta);
cpsi = cos(psi); spsi = sin(psi);
R = [ctheta*cpsi, ctheta*spsi, -stheta; ...
     sphi*stheta*cpsi-cphi*spsi, sphi*stheta*spsi+cphi*cpsi, sphi*ctheta; ...
     cphi*stheta*cpsi+sphi*spsi, cphi*stheta*spsi-sphi*cpsi, cphi*ctheta];
end

function segments = buildSegments(T, params, Ts)
gap = [false; diff(double(T.timestamp_us)) > 100000];
starts = find(gap | [true; false(height(T)-1, 1)]);
ends = [starts(2:end) - 1; height(T)];
segments = struct([]);

continuousFields = {'roll_deg', 'pitch_deg', 'yaw_deg', ...
    'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps', ...
    'vel_est_m_s_0', 'vel_est_m_s_1', 'vel_est_m_s_2', ...
    'ctrl_accel_out_m_s2_0', 'ctrl_accel_out_m_s2_1', 'ctrl_accel_out_m_s2_2', ...
    'ctrl_force_cmd_n_0', 'ctrl_force_cmd_n_1', 'ctrl_force_cmd_n_2'};
heldFields = {'ctrl_tilt_out_rad_0', 'ctrl_tilt_out_rad_1', ...
    'servo_alpha_us', 'servo_beta_us', 'motor_upper_us', 'motor_lower_us'};

for k = 1:numel(starts)
    rows = starts(k):ends(k);
    active = T.imu_valid(rows) ~= 0 & T.motor_output_reason(rows) == 1;
    rows = rows(active);
    if numel(rows) < 300
        continue;
    end
    seq = double(T.sequence(rows));
    seqGrid = (seq(1):seq(end))';
    if numel(seqGrid) < 300
        continue;
    end
    item = struct();
    item.sequence = seqGrid;
    item.time_s = (seqGrid - seqGrid(1)) * Ts;
    for f = 1:numel(continuousFields)
        name = continuousFields{f};
        values = double(T.(name)(rows));
        if strcmp(name, 'yaw_deg')
            values = rad2deg(unwrap(deg2rad(values)));
        end
        item.(name) = interp1(seq, values, seqGrid, 'linear', 'extrap');
    end
    for f = 1:numel(heldFields)
        name = heldFields{f};
        item.(name) = interp1(seq, double(T.(name)(rows)), seqGrid, 'previous', 'extrap');
    end
    paramNames = fieldnames(params);
    for f = 1:numel(paramNames)
        name = paramNames{f};
        item.(name) = interp1(seq, params.(name)(rows), seqGrid, 'previous', 'extrap');
    end
    item.saturation_fraction = mean( ...
        abs(item.ctrl_tilt_out_rad_0) >= 0.313 | ...
        abs(item.ctrl_tilt_out_rad_1) >= 0.313);
    item.source_rows = numel(rows);
    item.interpolated_samples = numel(seqGrid);
    item.motor_force_cmd_n = pwmToTotalForce(item.motor_upper_us, item.motor_lower_us);
    item.motor_force_actual_n = firstOrder(item.motor_force_cmd_n, Ts, 0.176624);
    if isempty(segments)
        segments = item;
    else
        segments(end + 1) = item; %#ok<AGROW>
    end
end
end

function experiments = buildAxisExperiments(segments, cfg, Ts)
experiments = struct([]);
for k = 1:numel(segments)
    s = segments(k);
    if s.saturation_fraction > 0.10
        continue;
    end
    angle = deg2rad(s.(cfg.angle_column));
    rate = cfg.rate_sign * deg2rad(s.(cfg.gyro_column));
    command = s.(cfg.tilt_column);
    drive = s.motor_force_actual_n * cfg.lever_m / 0.051 .* sin(command);
    angle = detrend(angle, 0);
    rate = detrend(rate, 0);
    drive = detrend(drive, 0);
    commandCentered = detrend(command, 0);
    if std(commandCentered) < deg2rad(0.3)
        continue;
    end
    y = [angle, rate];
    data = iddata(y, drive, Ts, ...
        'InputName', 'torque acceleration command', ...
        'OutputName', {[cfg.name ' angle'], [cfg.name ' rate']});
    item = struct( ...
        'data', data, 'angle', angle, 'rate', rate, ...
        'command', commandCentered, 'drive', drive, ...
        'samples', numel(angle), ...
        'saturation_fraction', s.saturation_fraction, ...
        'angle_kp', median(s.([cfg.name '_angle_kp']), 'omitnan'), ...
        'rate_kd', median(s.([cfg.name '_rate_kd']), 'omitnan'));
    if isempty(experiments)
        experiments = item;
    else
        experiments(end + 1) = item; %#ok<AGROW>
    end
end
end

function feedback = detectFeedback(experiments, Ts)
detected = false(1, numel(experiments));
zeroLag = nan(1, numel(experiments));
for k = 1:numel(experiments)
    e = experiments(k);
    tt = timetable(seconds((0:e.samples-1)' * Ts), e.command, e.angle, ...
        'VariableNames', {'servo_command', 'attitude_angle'});
    try
        [fbck, fbck0] = checkFeedback(tt, ...
            'InputName', 'servo_command', 'OutputName', 'attitude_angle');
        detected(k) = any(fbck(:) ~= 0);
        if isnumeric(fbck0) || islogical(fbck0)
            zeroLag(k) = double(any(fbck0(:) ~= 0));
        end
    catch error
        fprintf('checkFeedback skipped on experiment %d: %s\n', k, error.message);
    end
end
feedback = struct( ...
    'tested_count', numel(experiments), ...
    'detected_count', nnz(detected), ...
    'detected_by_experiment', detected, ...
    'zero_lag_flag_by_experiment', zeroLag, ...
    'interpretation', ['A detected loop means the actuator command is correlated ', ...
                       'with output noise; an innovation disturbance model is required.']);
end

function result = identifyDelayFamily(experiments, cfg)
delays = [0.04, 0.06, 0.08, 0.10];
delayResults = struct([]);
for delay = delays
    fprintf('  delay %.0f ms: ', 1000 * delay);
    angleFits = [];
    rateFits = [];
    foldParams = [];
    for fold = 1:2
        if fold == 1
            trainIndices = 1:2:numel(experiments);
            validationIndices = 2:2:numel(experiments);
        else
            trainIndices = 2:2:numel(experiments);
            validationIndices = 1:2:numel(experiments);
        end
        if isempty(trainIndices) || isempty(validationIndices)
            continue;
        end
        training = mergeExperimentData(experiments, trainIndices);
        model = fitGreybox(training, cfg, delay);
        foldParams = [foldParams; greyboxParameters(model)]; %#ok<AGROW>
        for index = validationIndices
            try
                [~, fit] = compare(experiments(index).data, model, ...
                    compareOptions('InitialCondition', 'e'));
                angleFits(end + 1) = fit(1); %#ok<AGROW>
                rateFits(end + 1) = fit(2); %#ok<AGROW>
            catch error
                fprintf('compare failed: %s ', error.message);
            end
        end
    end
    item = struct( ...
        'delay_s', delay, ...
        'validation_angle_fit_percent', median(angleFits, 'omitnan'), ...
        'validation_rate_fit_percent', median(rateFits, 'omitnan'), ...
        'validation_angle_fits_percent', angleFits, ...
        'validation_rate_fits_percent', rateFits, ...
        'fold_parameters', foldParams);
    item.validation_score_percent = median([angleFits, rateFits], 'omitnan');
    fprintf('angle %.1f%%, rate %.1f%%\n', ...
            item.validation_angle_fit_percent, item.validation_rate_fit_percent);
    if isempty(delayResults)
        delayResults = item;
    else
        delayResults(end + 1) = item; %#ok<AGROW>
    end
end

[~, bestIndex] = max([delayResults.validation_score_percent]);
best = delayResults(bestIndex);
allData = mergeExperimentData(experiments, 1:numel(experiments));
finalModel = fitGreybox(allData, cfg, best.delay_s);
finalParams = greyboxParameters(finalModel);
try
    covariance = getcov(finalModel);
    standardDeviation = sqrt(max(0, diag(covariance)));
    standardDeviation = standardDeviation(1:min(4, numel(standardDeviation)));
catch
    standardDeviation = nan(4, 1);
end

result = struct( ...
    'method', ['greyest prediction-error method with DisturbanceModel=estimate; ', ...
               'two-fold validation holds out complete experiment segments'], ...
    'delay_family_s', delays, ...
    'delay_results', delayResults, ...
    'best_delay_s', best.delay_s, ...
    'validation_angle_fit_percent', best.validation_angle_fit_percent, ...
    'validation_rate_fit_percent', best.validation_rate_fit_percent, ...
    'final_parameters', struct( ...
        'servo_tau_s', finalParams(1), ...
        'effectiveness', finalParams(2), ...
        'passive_damping_s1', finalParams(3), ...
        'tether_stiffness_s2', finalParams(4)), ...
    'parameter_standard_deviation', standardDeviation, ...
    'free_flight_conversion', ['Tether stiffness is set to zero and passive ', ...
        'damping is not credited during robust tuning.']);
end

function data = mergeExperimentData(experiments, indices)
data = experiments(indices(1)).data;
for k = indices(2:end)
    data = merge(data, experiments(k).data);
end
end

function model = fitGreybox(data, cfg, delay)
parameters = {0.04, cfg.plant_sign * cfg.previous_effectiveness, 1.0, 10.0};
model0 = idgrey(@attitudeGreybox, parameters, 'c');
model0.InputDelay = delay;
model0.InputName = 'torque acceleration command';
model0.OutputName = {[cfg.name ' angle'], [cfg.name ' rate']};

model0.Structure.Parameters(1).Minimum = 0.005;
model0.Structure.Parameters(1).Maximum = 0.200;
if cfg.plant_sign > 0
    model0.Structure.Parameters(2).Minimum = 0.05;
    model0.Structure.Parameters(2).Maximum = 1.50;
else
    model0.Structure.Parameters(2).Minimum = -1.50;
    model0.Structure.Parameters(2).Maximum = -0.05;
end
model0.Structure.Parameters(3).Minimum = 0.0;
model0.Structure.Parameters(3).Maximum = 30.0;
model0.Structure.Parameters(4).Minimum = 0.0;
model0.Structure.Parameters(4).Maximum = 200.0;

allOutputs = cell2mat(cellfun(@(x) x.OutputData, ...
    arrayfun(@(k) getexp(data, k), 1:size(data, 'Ne'), 'UniformOutput', false), ...
    'UniformOutput', false)');
scale = std(allOutputs, 0, 1);
scale(scale < 1e-6) = 1;
options = greyestOptions( ...
    'DisturbanceModel', 'estimate', ...
    'Focus', 'prediction', ...
    'InitialState', 'estimate', ...
    'EnforceStability', true, ...
    'OutputWeight', diag(1 ./ (scale .^ 2)), ...
    'Display', 'off');
options.SearchOptions.MaxIterations = 50;
options.SearchOptions.Tolerance = 1e-5;
model = greyest(data, model0, options);
end

function values = greyboxParameters(model)
values = zeros(1, 4);
for k = 1:4
    values(k) = model.Structure.Parameters(k).Value;
end
end

function [A, B, C, D, K, x0] = attitudeGreybox(tau, effectiveness, damping, stiffness, ~)
A = [0, 1, 0; ...
     -stiffness, -damping, effectiveness; ...
     0, 0, -1/tau];
B = [0; 0; 1/tau];
C = [1, 0, 0; 0, 1, 0];
D = zeros(2, 1);
K = zeros(3, 2);
x0 = zeros(3, 1);
end

function tuning = tuneControllerInLoop(cfg, identified)
warningState = warning;
warningCleanup = onCleanup(@() warning(warningState));
warning('off', 'all');
physics = struct( ...
    'mass', 1.367, 'gravity', 9.81, 'inertia', 0.051, ...
    'lever', cfg.lever_m, 'tilt_limit', deg2rad(18), ...
    'gyro_tau', 1/(2*pi*80), 'max_force', 17.5, ...
    'force_range_n', [5.5, 9.25, 13.40927, 16.85, 17.5]);
etaIdent = identified.final_parameters.effectiveness;
identificationValidated = identified.validation_angle_fit_percent > 0 && ...
                          identified.validation_rate_fit_percent > 0;
if ~identificationValidated || ~isfinite(etaIdent) || ...
        sign(etaIdent) ~= cfg.plant_sign || abs(etaIdent) < 0.10 || ...
        abs(etaIdent) >= 1.49
    etaNom = cfg.plant_sign * cfg.previous_effectiveness;
    etaSource = ['measured-PWM grey-box fallback because whole-experiment ', ...
                 'closed-loop validation failed'];
else
    etaNom = etaIdent;
    etaSource = 'closed-loop grey-box identification';
end
tauNom = identified.final_parameters.servo_tau_s;
if ~identificationValidated
    tauNom = cfg.previous_tau_s;
    delayNom = cfg.previous_delay_s;
else
    tauNom = min(max(tauNom, 0.01), 0.15);
    delayNom = identified.best_delay_s;
end
forceNom = physics.mass * physics.gravity;

kpGrid = -5.0:0.25:5.0;
kdGrid = -1.00:0.025:-0.025;
nominal = struct([]);
for kp = kpGrid
    for kd = kdGrid
        metrics = linearMetrics(cfg, kp, kd, etaNom, forceNom, ...
                                physics.inertia, tauNom, delayNom, physics);
        if metrics.stable && metrics.phase_margin_deg >= 10 && ...
                metrics.gain_margin_db >= 6 && ...
                metrics.crossover_rad_s >= 0.4 && metrics.crossover_rad_s <= 8.0
            item = struct( ...
                'kp', kp, 'kd', kd, ...
                'nominal_pm', metrics.phase_margin_deg, ...
                'nominal_gm', metrics.gain_margin_db, ...
                'nominal_wc', metrics.crossover_rad_s, ...
                'score', abs(metrics.crossover_rad_s - 1.5) + ...
                         max(0, 60 - metrics.phase_margin_deg) / 30);
            if isempty(nominal)
                nominal = item;
            else
                nominal(end + 1) = item; %#ok<AGROW>
            end
        end
    end
end

tuning = struct();
tuning.current_firmware_linearization = controllerLinearizationDescription(cfg);
tuning.effectiveness_used = etaNom;
tuning.effectiveness_source = etaSource;
tuning.nominal_servo_tau_s = tauNom;
tuning.nominal_delay_s = delayNom;
tuning.search = struct( ...
    'kp_code_range_n_per_rad', [kpGrid(1), kpGrid(end)], ...
    'kd_code_range_n_m_s_per_rad', [kdGrid(1), kdGrid(end)], ...
        'target_crossover_rad_s', 1.5, ...
        'minimum_phase_margin_deg', 50, ...
        'minimum_gain_margin_db', 6, ...
        'controller_force_range_n', physics.force_range_n);

if isempty(nominal)
    tuning.candidate_found = false;
    tuning.failure_reason = 'No nominally stable gain pair met the search constraints.';
    tuning.diagnostic_points = diagnosticGainPoints(cfg, etaNom, tauNom, delayNom, physics);
    return;
end

[~, order] = sort([nominal.score]);
nominal = nominal(order);
robust = struct([]);
for k = 1:numel(nominal)
    item = robustMetrics(cfg, nominal(k).kp, nominal(k).kd, etaNom, ...
                         tauNom, physics);
    item.kp_code_n_per_rad = nominal(k).kp;
    item.kd_code_n_m_s_per_rad = nominal(k).kd;
    item.nominal_crossover_rad_s = nominal(k).nominal_wc;
    item.nominal_phase_margin_deg = nominal(k).nominal_pm;
    item.nominal_gain_margin_db = nominal(k).nominal_gm;
    item.score = abs(item.nominal_crossover_rad_s - 1.5) + ...
                 max(0, 60 - item.min_phase_margin_deg) / 20;
    if isempty(robust)
        robust = item;
    else
        robust(end + 1) = item; %#ok<AGROW>
    end
end

passedMask = [robust.all_models_stable] & ...
             [robust.min_phase_margin_deg] >= 50 & ...
             [robust.min_gain_margin_db] >= 6;
if ~any(passedMask)
    tuning.candidate_found = false;
    tuning.failure_reason = ['Nominal candidates failed at least one uncertainty ', ...
                             'corner (delay, servo tau, inertia, force, or effectiveness).'];
    stableIndices = find([robust.all_models_stable] & ...
        isfinite([robust.min_phase_margin_deg]) & ...
        isfinite([robust.min_gain_margin_db]) & ...
        [robust.min_gain_margin_db] >= 6);
    if isempty(stableIndices)
        tuning.best_failed_candidates = robust(1:min(10, numel(robust)));
    else
        [~, marginOrder] = sort([robust(stableIndices).min_phase_margin_deg], 'descend');
        stableIndices = stableIndices(marginOrder);
        tuning.best_failed_candidates = ...
            robust(stableIndices(1:min(20, numel(stableIndices))));
    end
    assert(~isempty(stableIndices), ...
        'No stable commissioning candidate exists over measured force range.');
    commissioning = robust(stableIndices(1));
    commissioningKp = commissioning.kp_code_n_per_rad;
    commissioningKd = commissioning.kd_code_n_m_s_per_rad;
    nominalMetrics = linearMetrics(cfg, commissioningKp, commissioningKd, ...
        etaNom, forceNom, physics.inertia, tauNom, delayNom, physics);
    commissioning.kp_code_n_per_rad = commissioningKp;
    commissioning.kd_code_n_m_s_per_rad = commissioningKd;
    commissioning.synex_ui_kp = -commissioningKp;
    commissioning.synex_ui_kd = -commissioningKd;
    commissioning.nominal_crossover_rad_s = nominalMetrics.crossover_rad_s;
    commissioning.nominal_phase_margin_deg = nominalMetrics.phase_margin_deg;
    commissioning.nominal_gain_margin_db = nominalMetrics.gain_margin_db;
    commissioning.rigid_body_wn_rad_s = sqrt(max(0, ...
        abs(etaNom) * cfg.lever_m * ...
        (forceNom + commissioningKp) / physics.inertia));
    commissioning.rigid_body_zeta = abs(etaNom * commissioningKd) / ...
        max(1e-6, 2 * physics.inertia * commissioning.rigid_body_wn_rad_s);
    commissioning.nonlinear_nominal = nonlinearCheck(cfg, ...
        commissioningKp, commissioningKd, etaNom, forceNom, physics.inertia, ...
        tauNom, delayNom, physics);
    commissioning.nonlinear_worst_delay = nonlinearCheck(cfg, ...
        commissioningKp, commissioningKd, etaNom * 1.2, forceNom * 1.1, ...
        physics.inertia * 0.9, min(0.20, tauNom * 1.5), 0.10, physics);
    commissioning.nonlinear_low_force = nonlinearCheck(cfg, ...
        commissioningKp, commissioningKd, etaNom * 0.8, ...
        physics.force_range_n(1), physics.inertia * 1.1, ...
        min(0.20, tauNom * 1.5), 0.10, physics);
    commissioning.meets_required_margin = ...
        commissioning.all_models_stable && ...
        commissioning.min_phase_margin_deg >= 50 && ...
        commissioning.min_gain_margin_db >= 6;
    commissioning.usage = ['Restrained low-thrust commissioning start only; ', ...
        'not validated for free flight.'];
    tuning.commissioning_candidate = commissioning;
    tuning.diagnostic_points = diagnosticGainPoints(cfg, etaNom, tauNom, delayNom, physics);
    return;
end
[~, robustOrder] = sort([robust.score]);
robust = robust(robustOrder);
passed = find([robust.all_models_stable] & ...
              [robust.min_phase_margin_deg] >= 50 & ...
              [robust.min_gain_margin_db] >= 6);
[~, selectedLocal] = min([robust(passed).score]);
selected = robust(passed(selectedLocal));
selected.synex_ui_kp = -selected.kp_code_n_per_rad;
selected.synex_ui_kd = -selected.kd_code_n_m_s_per_rad;
selected.rigid_body_wn_rad_s = sqrt(max(0, ...
    abs(etaNom) * cfg.lever_m * ...
    (forceNom + selected.kp_code_n_per_rad) / physics.inertia));
selected.rigid_body_zeta = abs(etaNom * selected.kd_code_n_m_s_per_rad) / ...
    max(1e-6, 2 * physics.inertia * selected.rigid_body_wn_rad_s);
selected.nonlinear_nominal = nonlinearCheck(cfg, selected.kp_code_n_per_rad, ...
    selected.kd_code_n_m_s_per_rad, etaNom, forceNom, physics.inertia, ...
    tauNom, delayNom, physics);
selected.nonlinear_worst_delay = nonlinearCheck(cfg, selected.kp_code_n_per_rad, ...
    selected.kd_code_n_m_s_per_rad, etaNom * 1.2, forceNom * 1.1, ...
    physics.inertia * 0.9, min(0.20, tauNom * 1.5), 0.10, physics);

tuning.optimization_candidate_found = selected.nonlinear_nominal.stable && ...
                                      selected.nonlinear_worst_delay.stable;
tuning.physical_candidate_found = tuning.optimization_candidate_found;
tuning.model_validation_passed = ...
    identified.validation_angle_fit_percent > 0 && ...
    identified.validation_rate_fit_percent > 0;
tuning.candidate_found = tuning.optimization_candidate_found && ...
                         tuning.model_validation_passed;
tuning.recommended = selected;
tuning.top_robust_candidates = robust(passed(1:min(10, numel(passed))));
tuning.diagnostic_points = diagnosticGainPoints(cfg, etaNom, tauNom, delayNom, physics);
tuning.safety_status = ['Simulation candidate only. The identification data were ', ...
    'captured with a carbon-rod constraint and no measured servo position.'];
if ~tuning.model_validation_passed
    tuning.failure_reason = ['A mathematical optimization candidate exists, but ', ...
        'whole-experiment closed-loop validation fit is not positive on both ', ...
        'angle and rate; migration to hardware is blocked.'];
elseif ~tuning.optimization_candidate_found
    tuning.failure_reason = 'The nonlinear initial-angle checks did not converge.';
end
clear warningCleanup;
end

function textValue = controllerLinearizationDescription(cfg)
if strcmp(cfg.name, 'pitch')
    textValue = ['Corrected source: Ctheta=-(T+Kp)/T with ', ...
                 'pitch frame sign +1; Cq=Kd/(T*lever).'];
else
    textValue = ['Corrected source: Ctheta=(T+Kp)/T with roll frame sign -1; ', ...
                 'canonical roll rate is ', ...
                 '-gyro_x, therefore Cq=-Kd/(T*lever).'];
end
end

function metrics = linearMetrics(cfg, kp, kd, eta, force, inertia, tau, delay, physics)
if strcmp(cfg.name, 'pitch')
    cTheta = -(force + kp) / force;
    cRate = kd / (force * cfg.lever_m);
else
    cTheta = (force + kp) / force;
    cRate = -kd / (force * cfg.lever_m);
end
C = tf([cRate + cTheta * physics.gyro_tau, cTheta], ...
       [physics.gyro_tau, 1]);
P = tf(eta * force * cfg.lever_m / inertia, ...
       conv([tau, 1], [1, 0, 0]), 'InputDelay', delay);
Papprox = pade(P, 4);
closedLoop = feedback(Papprox, -C);
loop = -Papprox * C;
metrics.stable = isstable(closedLoop);
if ~metrics.stable
    metrics.gain_margin_db = -Inf;
    metrics.phase_margin_deg = -Inf;
    metrics.crossover_rad_s = 0;
    return;
end
try
    [gm, pm, wcg, wcp] = margin(loop);
    metrics.gain_margin_db = gainMarginDb(gm);
    metrics.phase_margin_deg = finiteOr(pm, -Inf);
    if isfinite(wcp) && wcp > 0
        metrics.crossover_rad_s = wcp;
    elseif isfinite(wcg) && wcg > 0
        metrics.crossover_rad_s = wcg;
    else
        metrics.crossover_rad_s = 0;
    end
catch
    metrics.gain_margin_db = -Inf;
    metrics.phase_margin_deg = -Inf;
    metrics.crossover_rad_s = 0;
end
end

function result = robustMetrics(cfg, kp, kd, etaNom, tauNom, physics)
etaValues = etaNom * [0.8, 1.2];
forceValues = physics.force_range_n;
inertiaValues = physics.inertia * [0.9, 1.1];
tauValues = unique([max(0.01, tauNom * 0.5), min(0.20, tauNom * 1.5)]);
delayValues = [0.04, 0.06, 0.08, 0.10];
minPm = Inf;
minGm = Inf;
stable = true;
models = 0;
for eta = etaValues
    for force = forceValues
        for inertia = inertiaValues
            for tau = tauValues
                for delay = delayValues
                    m = linearMetrics(cfg, kp, kd, eta, force, inertia, tau, delay, physics);
                    models = models + 1;
                    stable = stable && m.stable;
                    minPm = min(minPm, m.phase_margin_deg);
                    minGm = min(minGm, m.gain_margin_db);
                end
            end
        end
    end
end
result = struct( ...
    'all_models_stable', stable, ...
    'models_checked', models, ...
    'min_phase_margin_deg', minPm, ...
    'min_gain_margin_db', minGm, ...
    'uncertainty', struct( ...
        'effectiveness_scale', [0.8, 1.2], ...
        'force_range_n', forceValues, ...
        'inertia_scale', [0.9, 1.1], ...
        'servo_tau_scale', [0.5, 1.5], ...
        'delay_s', delayValues));
end

function points = diagnosticGainPoints(cfg, eta, tau, delay, physics)
pairs = [0, -0.5; -5, -1; -50, -1; 5, -1; 15, -0.5];
points = struct([]);
force = physics.mass * physics.gravity;
for k = 1:size(pairs, 1)
    m = linearMetrics(cfg, pairs(k,1), pairs(k,2), eta, force, ...
                      physics.inertia, tau, delay, physics);
    item = struct( ...
        'kp_code_n_per_rad', pairs(k,1), ...
        'kd_code_n_m_s_per_rad', pairs(k,2), ...
        'stable', m.stable, ...
        'phase_margin_deg', m.phase_margin_deg, ...
        'gain_margin_db', m.gain_margin_db, ...
        'crossover_rad_s', m.crossover_rad_s);
    if isempty(points)
        points = item;
    else
        points(end + 1) = item; %#ok<AGROW>
    end
end
end

function result = nonlinearCheck(cfg, kp, kd, eta, force, inertia, tau, delay, physics)
dt = 0.001;
duration = 8.0;
count = round(duration / dt) + 1;
theta = zeros(count, 1);
rate = zeros(count, 1);
servo = zeros(count, 1);
command = zeros(count, 1);
filteredRawRate = 0;
theta(1) = deg2rad(10);
delaySamples = max(0, round(delay / dt));
for k = 1:count-1
    if strcmp(cfg.name, 'pitch')
        forceX = -force * sin(theta(k)) - kp * theta(k);
        forceZ = max(1e-4, force * cos(theta(k)));
        rawRate = rate(k);
        ff = atan2(forceX, forceZ);
    else
        forceY = -force * sin(theta(k)) - kp * theta(k);
        forceZ = max(1e-4, force * cos(theta(k)));
        rawRate = -rate(k);
        ff = -atan2(forceY, forceZ);
    end
    filteredRawRate = filteredRawRate + ...
        dt / (physics.gyro_tau + dt) * (rawRate - filteredRawRate);
    rateTerm = kd * filteredRawRate / max(1e-6, forceZ * cfg.lever_m);
    command(k) = min(max(ff + rateTerm, -physics.tilt_limit), physics.tilt_limit);
    delayedIndex = max(1, k - delaySamples);
    delayedCommand = command(delayedIndex);
    servo(k+1) = servo(k) + dt / max(tau, dt) * (delayedCommand - servo(k));
    acceleration = eta * force * cfg.lever_m / inertia * sin(servo(k));
    rate(k+1) = rate(k) + dt * acceleration;
    theta(k+1) = theta(k) + dt * rate(k);
end
command(end) = command(end-1);
absTheta = abs(theta);
settled = absTheta <= deg2rad(0.5) & abs(rate) <= deg2rad(2);
settlingTime = NaN;
for k = 1:count
    if settled(k) && all(settled(k:end))
        settlingTime = (k-1) * dt;
        break;
    end
end
result = struct( ...
    'stable', all(isfinite(theta)) && max(absTheta) < deg2rad(45) && ...
              absTheta(end) < deg2rad(1.0), ...
    'initial_angle_deg', 10, ...
    'final_angle_deg', rad2deg(theta(end)), ...
    'peak_angle_deg', rad2deg(max(absTheta)), ...
    'peak_command_deg', rad2deg(max(abs(command))), ...
    'command_saturation_fraction', mean(abs(command) >= physics.tilt_limit - 1e-6), ...
    'settling_time_s', settlingTime, ...
    'time_s', (0:count-1)' * dt, ...
    'angle_deg', rad2deg(theta), ...
    'servo_command_deg', rad2deg(command));
end

function value = gainMarginDb(gainMargin)
if isinf(gainMargin)
    value = 300;
elseif isempty(gainMargin) || ~isfinite(gainMargin) || gainMargin <= 0
    value = -Inf;
else
    value = 20 * log10(gainMargin);
end
end

function value = finiteOr(value, fallback)
if isempty(value) || ~isfinite(value)
    value = fallback;
end
end

function forceN = pwmToTotalForce(upperUs, lowerUs)
pwm = [1100, 1142, 1184, 1226, 1268, 1310, 1352, 1394, 1436, 1478, ...
       1520, 1562, 1604, 1646, 1688, 1730, 1772, 1814, 1856, 1898, 1940];
dualThrustG = [0.000, 5.069, 25.589, 60.655, 106.361, 165.084, 216.696, ...
       287.758, 386.724, 501.680, 624.697, 725.173, 828.680, 923.574, ...
       981.674, 1114.845, 1256.137, 1366.352, 1466.668, 1541.404, 1595.342];
upperG = interp1(pwm, dualThrustG, min(max(upperUs, pwm(1)), pwm(end)), 'linear');
lowerG = interp1(pwm, dualThrustG, min(max(lowerUs, pwm(1)), pwm(end)), 'linear');
forceN = (upperG + lowerG) / (2 * 101.971621);
end

function output = firstOrder(input, Ts, tau)
output = zeros(size(input));
output(1) = input(1);
alpha = 1 - exp(-Ts / tau);
for k = 2:numel(input)
    output(k) = output(k-1) + alpha * (input(k) - output(k-1));
end
end

function value = clamp(value, lower, upper)
value = min(max(value, lower), upper);
end

function savePlots(outDir, results)
figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 440]);
for axisIndex = 1:2
    if axisIndex == 1
        axisName = 'pitch';
    else
        axisName = 'roll';
    end
    subplot(1, 2, axisIndex);
    id = results.(axisName).closed_loop_identification;
    delays = 1000 * [id.delay_results.delay_s];
    angleFit = [id.delay_results.validation_angle_fit_percent];
    rateFit = [id.delay_results.validation_rate_fit_percent];
    plot(delays, angleFit, '-o', 'LineWidth', 1.5); hold on;
    plot(delays, rateFit, '-s', 'LineWidth', 1.5);
    grid on; xlabel('Fixed delay (ms)'); ylabel('Validation fit (%)');
    title(upper(axisName)); legend('Angle', 'Rate', 'Location', 'best');
end
exportgraphics(gcf, fullfile(outDir, 'delay_validation.png'), 'Resolution', 150);
close(gcf);

figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 440]);
for axisIndex = 1:2
    if axisIndex == 1
        axisName = 'pitch';
    else
        axisName = 'roll';
    end
    subplot(1, 2, axisIndex);
    tuning = results.(axisName).controller_in_loop_tuning;
    if tuning.candidate_found || ...
            (isfield(tuning, 'physical_candidate_found') && tuning.physical_candidate_found)
        n = tuning.recommended.nonlinear_nominal;
        w = tuning.recommended.nonlinear_worst_delay;
        plot(n.time_s, n.angle_deg, 'LineWidth', 1.5); hold on;
        plot(w.time_s, w.angle_deg, '--', 'LineWidth', 1.5);
        legend('Nominal', 'Worst delay/gain', 'Location', 'best');
    else
        text(0.5, 0.5, 'No robust candidate', 'HorizontalAlignment', 'center');
    end
    grid on; xlabel('Time (s)'); ylabel('Angle (deg)'); title(upper(axisName));
end
exportgraphics(gcf, fullfile(outDir, 'nonlinear_initial_angle.png'), 'Resolution', 150);
close(gcf);
end

function writeReport(path, results)
fid = fopen(path, 'w', 'n', 'UTF-8');
assert(fid >= 0, 'Cannot open report.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Closed-loop Attitude Identification\n\n');
fprintf(fid, 'Source: `%s`\n\n', results.source_csv);
q = results.data_quality;
fprintf(fid, '## Data and controller reconstruction\n\n');
fprintf(fid, '- Rows: %d; sequence coverage: %.2f%%; missing records: %d.\n', ...
    q.rows, 100*q.sequence_coverage_fraction, q.missing_records);
fprintf(fid, '- Complete experiment segments: %d; total tilt saturation: %.2f%%.\n', ...
    q.continuous_segments, 100*q.tilt_saturation_fraction);
r = results.controller_reconstruction;
fprintf(fid, ['- Capture model: roll/pitch frame signs `%+.0f / %+.0f`, ' ...
              'attitude-force sign `%+.0f`, ' ...
              'legacy rate lever `%.3f m`.\n'], ...
    r.best.roll_frame_sign, r.best.pitch_frame_sign, ...
    r.best.attitude_force_sign, ...
    r.capture_model.shared_rate_lever_m);
fprintf(fid, '- Force RMSE X/Y/Z: `%.6f / %.6f / %.6f N`.\n', ...
    r.best.force_x_rmse_n, r.best.force_y_rmse_n, r.best.force_z_rmse_n);
fprintf(fid, '- Tilt output RMSE pitch/roll: `%.7f / %.7f rad`.\n\n', ...
    r.tilt.pitch_rmse_rad, r.tilt.roll_rmse_rad);
i = results.force_frame_interference;
fprintf(fid, ['- Old-to-corrected frame force delta RMS X/Y: `%.4f / %.4f N`; ' ...
              '95th percentile `%.4f / %.4f N`.\n'], ...
    i.force_delta_x_rms_n, i.force_delta_y_rms_n, ...
    i.force_delta_x_p95_abs_n, i.force_delta_y_p95_abs_n);
fprintf(fid, ['- Force delta / angle-P RMS ratio pitch/roll: `%.3f / %.3f`; ' ...
              'tilt delta RMS `%.3f / %.3f deg`.\n\n'], ...
    i.force_delta_to_p_rms_ratio_pitch, i.force_delta_to_p_rms_ratio_roll, ...
    i.tilt_delta_pitch_rms_deg, i.tilt_delta_roll_rms_deg);

for axisNameCell = {'pitch', 'roll'}
    axisName = axisNameCell{1};
    a = results.(axisName);
    id = a.closed_loop_identification;
    fprintf(fid, '## %s\n\n', upper(axisName));
    fprintf(fid, '- `checkFeedback`: %d/%d experiments detected feedback.\n', ...
        a.feedback_check.detected_count, a.feedback_check.tested_count);
    fprintf(fid, ['- Best fixed delay: `%.0f ms`; held-out angle/rate fit: ' ...
                  '`%.2f%% / %.2f%%`.\n'], ...
        1000*id.best_delay_s, id.validation_angle_fit_percent, ...
        id.validation_rate_fit_percent);
    p = id.final_parameters;
    fprintf(fid, ['- Final grey-box: servo tau `%.2f ms`, effectiveness `%+.4f`, ' ...
                  'constrained damping `%.4f 1/s`, tether stiffness `%.4f 1/s^2`.\n'], ...
        1000*p.servo_tau_s, p.effectiveness, p.passive_damping_s1, ...
        p.tether_stiffness_s2);
    previous = a.previous_simple_pidtuner;
    fprintf(fid, ['- Previous simplified PID Tuner comparison: Kp force `%.6f N/rad`, ' ...
                  'Kd `%.6f N*m*s/rad` (not directly transferable).\n'], ...
        previous.angle_kp_force_n_per_rad, previous.rate_kd_n_m_s_per_rad);
    tuning = a.controller_in_loop_tuning;
    if tuning.candidate_found || ...
            (isfield(tuning, 'physical_candidate_found') && tuning.physical_candidate_found)
        c = tuning.recommended;
        fprintf(fid, ['- Corrected-controller physical candidate: code Kp `%+.3f N/rad`, ' ...
                      'code Kd `%+.3f N*m*s/rad`.\n'], ...
            c.kp_code_n_per_rad, c.kd_code_n_m_s_per_rad);
        fprintf(fid, ['- Synex/PARAM UI values after sign conversion: Kp `%+.3f`, ' ...
                      'Kd `%+.3f`; rigid-body wn/zeta `%.3f / %.3f`.\n'], ...
            c.synex_ui_kp, c.synex_ui_kd, ...
            c.rigid_body_wn_rad_s, c.rigid_body_zeta);
        fprintf(fid, ['- Robust margins: min PM `%.2f deg`, min GM `%.2f dB`, ' ...
                      '%d corner models.\n'], ...
            c.min_phase_margin_deg, c.min_gain_margin_db, c.models_checked);
        fprintf(fid, ['- Nonlinear 10 deg test: nominal final `%.3f deg`, worst-delay ' ...
                      'final `%.3f deg`, peak command `%.2f / %.2f deg`.\n\n'], ...
            c.nonlinear_nominal.final_angle_deg, ...
            c.nonlinear_worst_delay.final_angle_deg, ...
            c.nonlinear_nominal.peak_command_deg, ...
            c.nonlinear_worst_delay.peak_command_deg);
        if ~tuning.model_validation_passed
            fprintf(fid, ['- Historical closed-loop validation did not pass; this ', ...
                'candidate comes from measured inertia/lever/force, measured-PWM ', ...
                'effectiveness, and actuator-delay uncertainty, not direct migration.\n\n']);
        end
    else
        fprintf(fid, '- No gain pair passed every robust and nonlinear constraint.\n');
        fprintf(fid, '- Reason: %s\n\n', tuning.failure_reason);
        if isfield(tuning, 'commissioning_candidate')
            c = tuning.commissioning_candidate;
            fprintf(fid, ['- Restrained commissioning candidate: internal Kp/Kd ' ...
                '`%+.3f / %+.3f`, Synex `%+.3f / %+.3f`.\n'], ...
                c.kp_code_n_per_rad, c.kd_code_n_m_s_per_rad, ...
                c.synex_ui_kp, c.synex_ui_kd);
            fprintf(fid, ['- Worst-corner PM/GM `%.2f deg / %.2f dB`; nominal ' ...
                'crossover `%.3f rad/s`; margin requirement passed `%d`.\n'], ...
                c.min_phase_margin_deg, c.min_gain_margin_db, ...
                c.nominal_crossover_rad_s, c.meets_required_margin);
            fprintf(fid, ['- Nonlinear 10 deg final angle nominal/worst-delay ' ...
                '`%.3f / %.3f deg`; settling time `%.3f / %.3f s`.\n\n'], ...
                c.nonlinear_nominal.final_angle_deg, ...
                c.nonlinear_worst_delay.final_angle_deg, ...
                c.nonlinear_nominal.settling_time_s, ...
                c.nonlinear_worst_delay.settling_time_s);
            fprintf(fid, ['- Low-force (%.2f N) nonlinear final angle/settling ' ...
                '`%.3f deg / %.3f s`; stable `%d`.\n\n'], ...
                c.uncertainty.force_range_n(1), ...
                c.nonlinear_low_force.final_angle_deg, ...
                c.nonlinear_low_force.settling_time_s, ...
                c.nonlinear_low_force.stable);
        end
    end
end

fprintf(fid, '## Transfer boundary\n\n');
fprintf(fid, ['This is a simulation candidate, not a flight-approved tune. The log was ' ...
    'recorded under a carbon-rod constraint, has no measured servo position, and the ' ...
    'capture firmware used a legacy shared `0.201 m` damping lever. Validate polarity ' ...
    'with motors disabled, then use a restrained low-thrust test before free flight.\n']);
clear cleanup;
end
