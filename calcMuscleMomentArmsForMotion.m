function [momentArmsAreWrong, momentArms, discontinuities, muscleNames, coordinateNames] = calcMuscleMomentArmsForMotion(modelFilename, motionFilenames, coordinateNames, muscleFilter, threshold, verbose, plotMomentArms, filterFrequency)
%CHECKMUSCLEMOMENTARM Computes moment arms of desired muscles around joints
%   and checks for smoothness of those during the motion
% modelFilename:
%   provide osim model
% motionFilenames:
%   provide motions (.mot file) for this model (e.g. results of inverse kinematics)
% coordinateNames:
%   provide joint names that should be moved into motion position
%   moment arms will be calculated around these joints
% muscleFilter:
%   provide texts that will be used as a filter - only muscles including
%   these texts will be checked
% threshold:
%   this is the threshold to detect discontinuities
%   set this as desired - default is 0.004
% verbose:
%   0 or 1 wheter to write output to command window
% plotMomentArms:
%   0 or 1 (default) to indicate wheter moment arms should be plotted
% filterFrequency:
%   frequency of iir filter applied to motion. non-positive values will be
%   ignored
%
% RETURN VALUES
% momentArmsAreWrong:
%   0 or 1 to indicate wheter moment arms have discontinuities
% momentArms:
%   cell array of moment arms for each motion - each element contains
%   array of momemt arms for respective input
%       dim 1: frames
%       dim 2: muscle number (to get the name use muscleNames variable)
%       dim 3: coordinate number (to get the name use coordinateNames variable)
% discontinuities:
%   cell array of moment arms for each motion - each element contains
%   array of discontinuities for respective input
%       dim 1: frame
%       dim 2: muscle number (to get the name use muscleNames variable)
%       dim 3: coordinate number (to get the name use coordinateNames variable)
% muscleNames:
%   cell array of muscle names
% coordinateNames:
%   cell array of coordinate names

import org.opensim.modeling.*
if ~exist('coordinateNames', 'var')
    coordinateNames = {'hip_flexion_l', 'hip_rotation_l', 'hip_adduction_l', ...
        'hip_flexion_r', 'hip_rotation_r', 'hip_adduction_r'}; %, ...
    % 'knee_flexion_l', 'knee_rotation_l', 'knee_adduction_l', ...
    % 'knee_flexion_r', 'knee_rotation_r', 'knee_adduction_r'};
end
if ~exist('muscleFilter', 'var')
    muscleFilter = {'add', 'gl', 'semi', 'bf', 'grac', 'piri', 'sart', 'tfl', 'iliacus', 'psoas', 'rect'};
end
if ~exist('threshold', 'var')
    threshold = 0.004;
end
if ~exist('verbose', 'var')
    verbose = 1;
end
if ~exist('plotMomentArms', 'var')
    plotMomentArms = 1;
end
if ~exist('filterFrequency', 'var')
    filterFrequency = 0;
end

model = Model(modelFilename);

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
        muscleIndices = [muscleIndices, i];
        muscleNames{end+1} = tmp_muscleName;
        muscleHandles{end+1} = model.getMuscles().get(i);
    end
end

coordSet = model.updCoordinateSet();

momentArmsAreWrong = 0;

% iterate through motion files
momentArms = cell(1, numel(motionFilenames));
discontinuities = cell(1, numel(motionFilenames));
for u = 1 : numel(motionFilenames)
    tic;
    motion = Storage(motionFilenames{u});
    disp(['Checking motion ' motionFilenames{u}]);

    if filterFrequency > 0
        motion.lowpassIIR(filterFrequency)
    end

    momentArmsCurrMotion = zeros(motion.getSize(), length(muscleIndices), numel(coordinateNames));
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
            fprintf(2, ['Following discontinuities were detected in file \n\t' strrep(motionFilenames{u}, '\', '/') '\n']);
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

    if plotMomentArms
        figure('Units','normalized', 'Position',[0 0.05 1 0.85]);
        tiledlayout('flow', 'TileSpacing','tight', 'Padding','tight');
        sgtitle([modelFilename ' - ' motionFilenames{u}], 'Interpreter', 'none');
        for i = 1 : numel(coordinateNames)
            nexttile;
            ylabel([coordinateNames{i} ' moment arm [m]'], 'Interpreter', 'none');
            xlabel('time [frame]');
            hold on;
            legendArr = {};
            for m = 1 : numel(muscleHandles)
                if abs(nansum(momentArmsCurrMotion(:, m, i))) > 1e-5
                    plot(momentArmsCurrMotion(:, m, i));
                    legendArr{end+1} = muscleNames{m};
                end
            end
            xlim([1 size(momentArmsCurrMotion, 1)]);

            % mark discontinuities
            if size(discontinuitiesCurrMotion, 1) > 0
                for d = 1 : size(discontinuitiesCurrMotion, 1)
                    if discontinuitiesCurrMotion(d, 3) == i
                        plot(discontinuitiesCurrMotion(d, 1), momentArmsCurrMotion(discontinuitiesCurrMotion(d, 1), ...
                            discontinuitiesCurrMotion(d, 2), discontinuitiesCurrMotion(d, 3)), ...
                            'rx', 'MarkerSize', 10, 'LineWidth', 2);
                        legendArr{end+1} = ['discontinuity ' muscleNames{discontinuitiesCurrMotion(d, 2)}];
                    end
                end
            end
            if size(discontinuitiesCurrMotion, 1) > 0
                musclesWithDiscont = unique(discontinuitiesCurrMotion(:, 2));
                muscleNamesWithDisc = muscleNames(musclesWithDiscont);
                legendArr(~contains(legendArr, muscleNamesWithDisc)) = '';
                if size(legendArr, 2) == 0
                    legend off;
                else
                    legend(legendArr, 'Interpreter', 'none', 'Location', 'best');
                end
            else
                legend off;
            end
        end
        drawnow;
    end
    momentArms{u} = momentArmsCurrMotion;
    discontinuities{u} = discontinuitiesCurrMotion;
end
end