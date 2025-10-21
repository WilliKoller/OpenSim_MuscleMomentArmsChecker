import org.opensim.modeling.*;

clear;

kinematicFolder = 'C:\Users\willi\ucloud\PhD\Study_LongitudinalMSK\SimulationOutput_Sanguex_GRF20Hz\TD06_S2_adjCond_modWO_opt_N10_final\gait03\Output\IK';
kinematicFileFilter = '*.mot';
modelFilename = 'C:\Users\willi\ucloud\PhD\Study_LongitudinalMSK\Models_Sangeux\TD06_S2_generic_final.osim';
% modelFilenameModified = strrep(modelFilename, '.osim', '_modWO.osim');
modelFilenameModified = 'C:\Users\willi\ucloud\PhD\Study_LongitudinalMSK\Models_Sangeux\OptimizedModels/TD06_S2_adjCond_modWO_opt_N10_final.osim';
% modelFilename = '';

motFiles = dir(fullfile(kinematicFolder, '**', kinematicFileFilter));
motionFileNames = cell(1, numel(motFiles));
for m = 1 : numel(motFiles)
    motionFileNames{m} = fullfile(motFiles(m).folder, motFiles(m).name);
end
filterFrequency = 0;

% for rajagopal or gait2392 model
% coordinateNames = {'hip_flexion_l', 'hip_rotation_l', 'hip_adduction_l', ...
%     'hip_flexion_r', 'hip_rotation_r', 'hip_adduction_r', ...
%     'knee_angle_l', 'knee_angle_r'};
% 
% % % for lernagopal
% coordinateNames = {'hip_flexion_l', 'hip_rotation_l', 'hip_adduction_l', ...
%     'hip_flexion_r', 'hip_rotation_r', 'hip_adduction_r', ...
%     'knee_angle_l', 'knee_angle_r', 'knee_adduction_l', 'knee_adduction_r', ...
%     'ankle_angle_l', 'ankle_angle_r', 'subtalar_angle_l', 'subtalar_angle_r'};
% % for lernagopal
coordinateNames = {'hip_flexion_r', 'hip_rotation_r', 'hip_adduction_r', ...
    'knee_angle_r', 'knee_adduction_r', 'ankle_angle_r', 'subtalar_angle_r'};

% specifiy muscle and coordinates that you want to plot
muscleFilter = {'_r'};

% muscleFilter = {'add', 'gl', 'semi', 'bf', 'pec', 'grac', 'piri', 'sar', ...
%     'tfl', 'iliacus', 'psoas', 'rect', 'gas', 'quad_fem', 'gem', 'peri', 'vas'};

plotCoordinates = coordinateNames;%(1:3);

% set factor to -1 if you want to reverse direction in plot
coordinateFactors = ones(1, size(plotCoordinates, 2));
coordinateFactors([1 2 3]) = -1;
plotCoordinatesNames = plotCoordinates;
plotCoordinatesNames{1} = 'hip extension_l';
plotCoordinatesNames{2} = 'hip external rotation_l';
plotCoordinatesNames{3} = 'hip abduction_l';

plotMomentArms = 1;
plotRMSDThreshold = 3e-3;
plotMuscleLength = 1;
plotRMSDLengthThreshold = 3e-3;
rowCount = ceil(sqrt(numel(plotCoordinatesNames)));
tileLayout = [rowCount, ceil(numel(plotCoordinatesNames) / rowCount)];
figureSize = [0 0.05 0.9 0.9]; % normalized to windows size
figureSizeMuscleLength = [0 0.05 0.6 0.6]; % normalized to windows size
verbose = 1;
threshold = 0.004;


% iterate through motion files
momentArms = cell(1, numel(motionFileNames));
for u = 1 : numel(motionFileNames)

    tic;
    motion = Storage(motionFileNames{u});
    disp(['Checking motion ' motionFileNames{u}]);

    if filterFrequency > 0
        motion.lowpassIIR(filterFrequency)
    end

    motionCoordinates = motion.getColumnLabels();
    motionCoordinateNames = [];
    for c = 0 : motionCoordinates.getSize - 1
        motionCoordinateNames{c+1} = char(motionCoordinates.get(c));
    end

    % run model 1
    model = Model(modelFilename);
    state = model.initSystem();

    coordInd = zeros(1, numel(coordinateNames));
    coordinateHandles = cell(1, numel(coordinateNames));
    for i = 1 : numel(coordinateNames)
        coordInd(i) = model.getCoordinateSet().getIndex(coordinateNames{i});
        coordinateHandles{i} = model.updCoordinateSet().get(coordInd(i));
    end

    numMuscles = model.getMuscles().getSize();
    muscleIndices = []; muscleNames = {};
    muscleHandles = {};
    for i = 0 : numMuscles - 1
        tmp_muscleName = char(model.getMuscles().get(i).getName());
        % find muscles that fulfill filter requirements
        if contains(tmp_muscleName, muscleFilter)
            % check if muscle with this id is the same in the modified model
            % tmp_muscleNameMod = char(modelModified.getMuscles().get(i).getName());
            muscleIndices = [muscleIndices, i];
            muscleNames{end+1} = tmp_muscleName;
            muscleHandles{end+1} = model.getMuscles().get(i);
        end
    end


    momentArmsCurrMotion = zeros(motion.getSize(), length(muscleIndices), numel(coordinateNames));
    muscleLengthCurrMotion = zeros(motion.getSize(), length(muscleIndices));
    for frame = 1:motion.getSize()
        % set all coordinates to values of the motion
        for i = 1 : numel(coordinateNames)
            motionIdx = find(ismember(motionCoordinateNames, coordinateNames{i}) == 1, 1);
            tmpAngle = motion.getStateVector(frame-1).getData().get(motionIdx - 2);
            if motion.isInDegrees
                tmpAngle = tmpAngle / 180 * pi;
            end
            coordinateHandles{i}.setValue(state, tmpAngle);
        end

        % Realize the state to compute dependent quantities
        model.computeStateVariableDerivatives(state);
        model.realizeVelocity(state);

        % iterate through muscles
        for m = 1 : numel(muscleHandles)
            % calculate moment arm around each coordinate for this muscle
            % if muscle is not spanning the joint, will be zero
            for i = 1 : numel(coordinateNames)
                momentArmsCurrMotion(frame, m, i) = muscleHandles{m}.computeMomentArm(state, coordinateHandles{i});
            end
            muscleLengthCurrMotion(frame, m) = muscleHandles{m}.getLength(state);
        end
    end

    momentArmsCurrMotionModel1 = momentArmsCurrMotion;
    muscleLengthCurrMotionModel1 = muscleLengthCurrMotion;
    %% run model 2
    model = Model(modelFilenameModified);
    model.initSystem();
    state = model.initSystem();

    coordInd = zeros(1, numel(coordinateNames));
    coordinateHandles = cell(1, numel(coordinateNames));
    for i = 1 : numel(coordinateNames)
        coordInd(i) = model.getCoordinateSet().getIndex(coordinateNames{i});
        coordinateHandles{i} = model.updCoordinateSet().get(coordInd(i));
    end

    numMuscles = model.getMuscles().getSize();
    muscleIndices = []; muscleNames = {};
    muscleHandles = {};
    for i = 0 : numMuscles - 1
        tmp_muscleName = char(model.getMuscles().get(i).getName());
        % find muscles that fulfill filter requirements
        if contains(tmp_muscleName, muscleFilter)
            % check if muscle with this id is the same in the modified model
            % tmp_muscleNameMod = char(modelModified.getMuscles().get(i).getName());
            muscleIndices = [muscleIndices, i];
            muscleNames{end+1} = tmp_muscleName;
            muscleHandles{end+1} = model.getMuscles().get(i);
        end
    end


    momentArmsCurrMotion = zeros(motion.getSize(), length(muscleIndices), numel(coordinateNames));
    muscleLengthCurrMotion = zeros(motion.getSize(), length(muscleIndices));
    for frame = 1:motion.getSize()
        % set all coordinates to values of the motion
        for i = 1 : numel(coordinateNames)
            motionIdx = find(ismember(motionCoordinateNames, coordinateNames{i}) == 1, 1);
            tmpAngle = motion.getStateVector(frame-1).getData().get(motionIdx - 2);
            if motion.isInDegrees
                tmpAngle = tmpAngle / 180 * pi;
            end
            coordinateHandles{i}.setValue(state, tmpAngle);
        end

        % Realize the state to compute dependent quantities
        model.computeStateVariableDerivatives(state);
        model.realizeVelocity(state);

        % iterate through muscles
        for m = 1 : numel(muscleHandles)
            % calculate moment arm around each coordinate for this muscle
            % if muscle is not spanning the joint, will be zero
            for i = 1 : numel(coordinateNames)
                momentArmsCurrMotion(frame, m, i) = muscleHandles{m}.computeMomentArm(state, coordinateHandles{i});
            end
            muscleLengthCurrMotion(frame, m) = muscleHandles{m}.getLength(state);
        end
    end
    toc

    momentArmsCurrMotionModified = momentArmsCurrMotion;
    muscleLengthCurrMotionModified = muscleLengthCurrMotion;


    momentArmsCurrMotion = momentArmsCurrMotionModel1;
    muscleLengthCurrMotion = muscleLengthCurrMotionModel1;

    %%

    numberOfMuscles = size(momentArmsCurrMotionModel1, 2);
    colors = parula(numberOfMuscles);

    if plotMomentArms
        figure('Units','normalized', 'Position',figureSize);
        tiledlayout(tileLayout(1), tileLayout(2), 'TileSpacing','tight', 'Padding','tight');
        sgtitle(['Visualization of muscle moment arms with a RMSD between models > ' num2str(plotRMSDThreshold)]);
        for i = 1 : numel(plotCoordinates)
            idxCoordinateHandle = find(contains(coordinateNames, plotCoordinates(i)));

            nexttile;
            ylabel([strrep(plotCoordinatesNames{i}(1:end-2), '_', ' ') ' moment arm [cm]'], 'Interpreter', 'none');
            xlabel('% gait cycle');
            hold on;
            legendArr = [];
            for m = 1 : numel(muscleHandles)
                rmsd = rmse(momentArmsCurrMotion(:, m, idxCoordinateHandle), momentArmsCurrMotionModified(:, m, idxCoordinateHandle));
                if abs(nansum(momentArmsCurrMotion(:, m, idxCoordinateHandle))) > 1e-5 && rmsd > plotRMSDThreshold
                    disp([plotCoordinates{i} ': ' muscleNames{m} ' RMSD = ' num2str(rmsd)]);
                    plot(0:100, normalizetimebase(momentArmsCurrMotion(:, m, idxCoordinateHandle)) * 100 * coordinateFactors(i), '-', 'color', colors(m, :), 'LineWidth', 1);
                    plot(0:100, normalizetimebase(momentArmsCurrMotionModified(:, m, idxCoordinateHandle)) * 100 * coordinateFactors(i), '--', 'color', colors(m, :), 'LineWidth', 2);
                    legendArr{end+1} = muscleNames{m};
                    legendArr{end+1} = [muscleNames{m} ' modified'];
                end
            end
            xlim([0 100]);
            ylims = ylim;
            if min(ylims(1)) > 0
                ylims(1) = 0;
            end
            if max(ylims(2)) < 0
                ylims(2) = 0;
            end
            ylim(ylims);

            legend(legendArr, 'Interpreter', 'none', 'location', 'best');

        end
        drawnow;
    end
    
    %%
    if plotMuscleLength
        figure('Units','normalized', 'Position',figureSizeMuscleLength);
        % tiledlayout(1, numel(muscleNames), 'TileSpacing','tight', 'Padding','tight');
        tiledlayout(1, 1);
        sgtitle(['Visualization of muscle lengths with a RMSD between models > ' num2str(plotRMSDLengthThreshold)]);
        nexttile;

        legendArr = [];
        ylabel('muscle length [cm]', 'Interpreter', 'none');
        xlabel('% gait cycle');
        hold on;
        for m = 1 : numel(muscleNames)
            rmsd = rmse(muscleLengthCurrMotion(:, m), muscleLengthCurrMotionModified(:, m));
            if rmsd > plotRMSDLengthThreshold
                plot(0:100, normalizetimebase(muscleLengthCurrMotion(:, m)) * 100, '-', 'color', colors(m, :), 'LineWidth', 1);
                plot(0:100, normalizetimebase(muscleLengthCurrMotionModified(:, m)) * 100, '--', 'color', colors(m, :), 'LineWidth', 2);
                legendArr{end+1} = muscleNames{m};
                legendArr{end+1} = [muscleNames{m} ' modified'];
            end

            % legend({'original model', 'modified model'}, 'Interpreter', 'none', 'Location', 'best');
        end
        xlim([0 100]);
        leg = legend(legendArr, 'Interpreter', 'none', 'location', 'best');
        leg.Layout.Tile = 'east';
        drawnow;
    end
end