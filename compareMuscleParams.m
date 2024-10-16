import org.opensim.modeling.*;

clear;

kinematicFolder = './ExampleData/kinematics';
kinematicFileFilter = '*.mot';
modelFilename = './ExampleData/model.osim';
modelFilenameModified = strrep(modelFilename, '.osim', '_modWO.osim');

motFiles = dir(fullfile(kinematicFolder, '**', kinematicFileFilter));
motionFileNames = cell(1, numel(motFiles));
for m = 1 : numel(motFiles)
    motionFileNames{m} = fullfile(motFiles(m).folder, motFiles(m).name);
end
filterFrequency = 6;

% for rajagopal or gait2392 model
coordinateNames = {'hip_flexion_l', 'hip_rotation_l', 'hip_adduction_l', ...
    'hip_flexion_r', 'hip_rotation_r', 'hip_adduction_r', ...
    'knee_angle_l', 'knee_angle_r'};

% % for lernagopal
% coordinateNames = {'hip_flexion_l', 'hip_rotation_l', 'hip_adduction_l', ...
%     'hip_flexion_r', 'hip_rotation_r', 'hip_adduction_r', ...
%     'knee_angle_l', 'knee_angle_r', 'knee_rotation_l', 'knee_rotation_r', 'knee_adduction_l', 'knee_adduction_r'};

% specifiy muscle and coordinates that you want to plot
muscleFilter = {'glmax1_l'};
plotCoordinates = coordinateNames(1:3);

% set factor to -1 if you want to reverse direction in plot
coordinateFactors = ones(1, size(plotCoordinates, 2));
coordinateFactors([1 2 3]) = -1;
plotCoordinatesNames = plotCoordinates;
plotCoordinatesNames{1} = 'hip extension_l';
plotCoordinatesNames{2} = 'hip external rotation_l';
plotCoordinatesNames{3} = 'hip abduction_l';

plotMomentArms = 1;
plotMuscleLength = 1;
tileLayout = [1, 3];
figureSize = [0 0.05 0.5 0.2]; % normalized to windows size
figureSizeMuscleLength = [0 0.05 0.5/3 0.2]; % normalized to windows size
verbose = 1;
threshold = 0.004;



% basically the function "calcMuscleMomentArmsForMotion", just modified to
% get nice plots
model = Model(modelFilename);
model.initSystem();
state = model.initSystem();

modelModified = Model(modelFilenameModified);
modelModified.initSystem();
stateModified = modelModified.initSystem();

coordInd = zeros(1, numel(coordinateNames));
coordinateHandles = cell(1, numel(coordinateNames));
coordinateHandlesModified = cell(1, numel(coordinateNames));
for i = 1 : numel(coordinateNames)
    coordInd(i) = model.getCoordinateSet().getIndex(coordinateNames{i});
    coordinateHandles{i} = model.updCoordinateSet().get(coordInd(i));
    coordinateHandlesModified{i} = modelModified.updCoordinateSet().get(coordInd(i));
end

numMuscles = model.getMuscles().getSize();
muscleIndices = []; muscleNames = {};
muscleHandles = {};
muscleHandlesModified = {};
for i = 0 : numMuscles - 1
    tmp_muscleName = char(model.getMuscles().get(i).getName());
    % find muscles that fulfill filter requirements
    if contains(tmp_muscleName, muscleFilter)
        muscleIndices = [muscleIndices, i];
        muscleNames{end+1} = tmp_muscleName;
        muscleHandles{end+1} = model.getMuscles().get(i);
        muscleHandlesModified{end+1} = modelModified.getMuscles().get(i);
    end
end

coordSet = model.updCoordinateSet();
coordSetModified = modelModified.updCoordinateSet();

momentArmsAreWrong = 0;

% iterate through motion files
momentArms = cell(1, numel(motionFileNames));
discontinuities = cell(1, numel(motionFileNames));
for u = 1 : numel(motionFileNames)
    tic;
    motion = Storage(motionFileNames{u});
    disp(['Checking motion ' motionFileNames{u}]);

    if filterFrequency > 0
        motion.lowpassIIR(filterFrequency)
    end

    momentArmsCurrMotion = zeros(motion.getSize(), length(muscleIndices), numel(coordinateNames));
    muscleLengthCurrMotion = zeros(motion.getSize(), length(muscleIndices));
    for frame = 1:motion.getSize()
        % set all coordinates to values of the motion
        for i = 1 : numel(coordinateNames)
            tmpAngle = motion.getStateVector(frame-1).getData().get(coordInd(i));
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
    momentArmsCurrMotionModified = zeros(motion.getSize(), length(muscleIndices), numel(coordinateNames));
    muscleLengthCurrMotionModified = zeros(motion.getSize(), length(muscleIndices));
    for frame = 1:motion.getSize()
        % set all coordinates to values of the motion
        for i = 1 : numel(coordinateNames)
            tmpAngle = motion.getStateVector(frame-1).getData().get(coordInd(i));
            if motion.isInDegrees
                tmpAngle = tmpAngle / 180 * pi;
            end
            coordinateHandlesModified{i}.setValue(stateModified, tmpAngle);
        end

        % Realize the state to compute dependent quantities
        modelModified.computeStateVariableDerivatives(stateModified);
        modelModified.realizeVelocity(stateModified);

        % iterate through muscles
        for m = 1 : numel(muscleHandles)
            % calculate moment arm around each coordinate for this muscle
            % if muscle is not spanning the joint, will be zero
            for i = 1 : numel(coordinateNames)
                momentArmsCurrMotionModified(frame, m, i) = muscleHandlesModified{m}.computeMomentArm(stateModified, coordinateHandlesModified{i});
            end
            muscleLengthCurrMotionModified(frame, m) = muscleHandlesModified{m}.getLength(stateModified);
        end
    end
    toc

    discontinuitiesCurrMotion = [];
    % check for discontinuities
    for i = 1 : numel(coordinateNames)
        for m = 1 : numel(muscleHandles)
            dy = diff(momentArmsCurrMotion(:, m, i));
            discontinuity_indices = find(abs(dy) > threshold);
            if size(discontinuity_indices, 1) > 0
                for d = 1 : size(discontinuity_indices, 1)
                    discontinuitiesCurrMotion(end+1, :) = [discontinuity_indices(d), m , i];
                end
            end
        end
    end

    if size(discontinuitiesCurrMotion, 1) > 0
        if verbose
            fprintf(2, ['Following discontinuities were detected in file \n\t' strrep(motionFileNames{u}, '\', '/') '\n']);
            for d = 1 : size(discontinuitiesCurrMotion, 1)
                fprintf(2,  [muscleNames{discontinuitiesCurrMotion(d, 2)} ' around ' coordinateNames{discontinuitiesCurrMotion(d, 3)} ' at ' num2str(motion.getStateVector(discontinuitiesCurrMotion(d, 1)).getTime) ' seconds (frame ' num2str(discontinuitiesCurrMotion(d, 1)) ')\n']);
            end
        else
            musclesWithDiscont = unique(discontinuitiesCurrMotion(:, 2));
            tmpText = 'Discontinuities detected for ';
            for m = 1 : numel(musclesWithDiscont)
                tmpText = [tmpText muscleNames{musclesWithDiscont(m)} ' '];
            end
            disp(tmpText);
        end
        momentArmsAreWrong = 1;
    end

    %%

    if plotMomentArms
        figure('Units','normalized', 'Position',figureSize);
        tiledlayout(tileLayout(1), tileLayout(2), 'TileSpacing','tight', 'Padding','tight');
        for i = 1 : numel(plotCoordinates)
            idxCoordinateHandle = find(contains(coordinateNames, plotCoordinates(i)));

            nexttile;
            ylabel([strrep(plotCoordinatesNames{i}(1:end-2), '_', ' ') ' moment arm [cm]'], 'Interpreter', 'none');
            xlabel('% gait cycle');
            hold on;
            for m = 1 : numel(muscleHandles)
                if abs(nansum(momentArmsCurrMotion(:, m, idxCoordinateHandle))) > 1e-5
                    plot(0:100, normalizetimebase(momentArmsCurrMotion(:, m, idxCoordinateHandle)) * 100 * coordinateFactors(i), 'color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
                    plot(0:100, normalizetimebase(momentArmsCurrMotionModified(:, m, idxCoordinateHandle)) * 100 * coordinateFactors(i), 'color', [0 0.4470 0.7410], 'LineWidth', 2);
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

            if i == 1
                legend({'original model', 'modified model'}, 'Interpreter', 'none', 'Location', 'best');
            end
        end
        drawnow;
    end
    
    if plotMuscleLength
        figure('Units','normalized', 'Position',figureSizeMuscleLength);
        tiledlayout(1, numel(muscleNames), 'TileSpacing','tight', 'Padding','tight');

        for m = 1 : numel(muscleNames)
            nexttile;
            ylabel([muscleNames{m} ' length [cm]'], 'Interpreter', 'none');
            xlabel('% gait cycle');
            hold on;

            plot(0:100, normalizetimebase(muscleLengthCurrMotion(:, m)) * 100, 'color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
            plot(0:100, normalizetimebase(muscleLengthCurrMotionModified(:, m)) * 100, 'color', [0 0.4470 0.7410], 'LineWidth', 2);

            xlim([0 100]);

            legend({'original model', 'modified model'}, 'Interpreter', 'none', 'Location', 'best');
        end
        drawnow;
    end
end