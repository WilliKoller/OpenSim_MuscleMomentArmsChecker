%-------------------------------------------------------------------------%
% Copyright (c) 2024 Koller W.                                            %
%    Author:   Willi Koller,  2024                                        %
%    email:    willi.koller@univie.ac.at                                  %
% ----------------------------------------------------------------------- %

% OpenSim_MuscleMomentArmsChecker (c) by Willi Koller, University of Vienna
%
% OpenSim_MuscleMomentArmsChecker is licensed under a
% Creative Commons Attribution-NonCommercial 4.0 International License.
%
% You should have received a copy of the license along with this
% work. If not, see <http://creativecommons.org/licenses/by-nc/4.0/>.

import org.opensim.modeling.*
clear; close all;

stepSize = 0.001; % value which is used to shrink wrap objects each iteration
threshold = 0.004; % value that defines the threshold for discontinuity detection+
filterFrequency = 6; % filter kinematic from .mot files; use -1 to disable

kinematicFolder = './ExampleData/kinematics';
kinematicFileFilter = '*.mot';
modelFilename = './ExampleData/model.osim';

motFiles = dir(fullfile(kinematicFolder, '**', kinematicFileFilter));
motionFileNames = cell(1, numel(motFiles));
for m = 1 : numel(motFiles)
    motionFileNames{m} = fullfile(motFiles(m).folder, motFiles(m).name);
end

[p, f] = fileparts(modelFilename);
delete(fullfile(p, [f '_modifyWrapObjects.log']));
diary(fullfile(p, [f '_modifyWrapObjects.log']));
figFilename = fullfile(p, [f '_momentArms.fig']);

% define the coordinates that are put into the position obtained from
% .mot file

% for rajagopal or gait2392 model
coordinateNames = {'hip_flexion_l', 'hip_rotation_l', 'hip_adduction_l', ...
    'hip_flexion_r', 'hip_rotation_r', 'hip_adduction_r', ...
    'knee_angle_l', 'knee_angle_r'};

% % for lernagopal
% coordinateNames = {'hip_flexion_l', 'hip_rotation_l', 'hip_adduction_l', ...
%     'hip_flexion_r', 'hip_rotation_r', 'hip_adduction_r', ...
%     'knee_angle_l', 'knee_angle_r', 'knee_rotation_l', 'knee_rotation_r', 'knee_adduction_l', 'knee_adduction_r'};

% to check for muscles that span the hip and knee joint
muscleFilter = {'add', 'gl', 'semi', 'bf', 'pec', 'grac', 'piri', 'sar', ...
    'tfl', 'iliacus', 'psoas', 'rect', 'gas', 'quad_fem', 'gem', 'peri', 'vas'};
% to check for muscles that span the hip and knee joint

% calc muscle moment arms once
[momentArmsAreWrong, ~, discontinuities, muscleNames, ~] = calcMuscleMomentArmsForMotion(modelFilename, ...
    motionFileNames, coordinateNames, muscleFilter, threshold, 1, 1, filterFrequency);

if momentArmsAreWrong
    newModelFilename = strrep(modelFilename, '.osim', '_modWO.osim');
    copyfile(modelFilename, newModelFilename);

    failed = 0;
    iteration = 1;
    wrapObjectsModified = {};
    wrapObjectsOrigRadius = [];
    wrapObjectsModifiedRadius = [];
    wrapEllipsoidsModified = {};
    wrapEllipsoidsOrigDimension = [];
    wrapEllipsoidsModifiedDimension = [];

    while(momentArmsAreWrong && iteration < 10)
        if iteration ~= 1
            for u = 1 : numel(discontinuities)
                close(gcf);
            end
            % close latest figures - they provide only temporary solutions
        end
        iteration = iteration + 1;

        model = Model(newModelFilename);
        model.initSystem();

        musclesWithDiscontinuities = {};
        for u = 1 : numel(discontinuities)
            if size(discontinuities{u}, 1) > 0
                for d = 1 : size(discontinuities{u}, 1)
                    musclesWithDiscontinuities{end+1} = muscleNames{discontinuities{u}(d, 2)};
                end
            end
        end

        musclesWithDiscontinuities = unique(musclesWithDiscontinuities);

        if size(musclesWithDiscontinuities, 1) > 0
            madeModifications = 0;
            disp(' ');
            disp('------------------------------------------------');
            disp(['Iteration #' num2str(iteration) ' these muscles have discontinuios moment arms:']);
            for m = 1 : numel(musclesWithDiscontinuities)
                muscleName = musclesWithDiscontinuities{m};
                disp(muscleName);
                muscle = model.getMuscles().get(muscleName);

                geoPath = muscle.getGeometryPath;
                wrapSet = geoPath.getWrapSet;
                for w = 1 : wrapSet.getSize()
                    wrapObj = wrapSet.get(w-1);
                    wrapObjName = wrapObj.getWrapObjectName;
                    for i = 1 : model.getBodySet.getSize
                        currBody = model.getBodySet.get(i-1);
                        for j = 1 : currBody.getWrapObjectSet.getSize
                            if strcmp(currBody.getWrapObjectSet.get(j-1).getName, wrapObjName)
                                wrapObject = currBody.getWrapObjectSet.get(j-1);
                                try
                                    wrapCylinder = WrapCylinder.safeDownCast(wrapObject);
                                    radius = wrapCylinder.get_radius;
                                    if radius - stepSize > 0
                                        wrapCylinder.set_radius(radius - stepSize)
                                        model.print(newModelFilename);
                                        pause(0.5);
                                        disp([char(wrapObjName) ': radius decreased to ' num2str(radius - stepSize)]);
                                        wrapObjectsModified{end+1} = char(wrapObjName);
                                        wrapObjectsOrigRadius(end+1) = radius;
                                        wrapObjectsModifiedRadius(end+1) = radius - stepSize;
                                        madeModifications = 1;
                                        break;
                                    else
                                        disp([muscleName ' moment arm wrong but wrap object is already too small! Check manually']);
                                        failed = 1;
                                        momentArmsAreWrong = 0;
                                    end
                                catch
                                    wrapEllipsoid = WrapEllipsoid.safeDownCast(wrapObject);
                                    dimensionsString = wrapEllipsoid.getDimensionsString;
                                    dimensions = strsplit(char(dimensionsString), ' ');
                                    newDimension(1) = str2double(dimensions(2))-stepSize;
                                    newDimension(2) = str2double(dimensions(3))-stepSize;
                                    newDimension(3) = str2double(dimensions(4))-stepSize;
                                    if sum(newDimension > 0) == 3
                                        newDimensionString = [num2str(newDimension(1)) ...
                                            ' ' num2str(newDimension(2)) ' ' num2str(newDimension(3))];

                                        %replace dimensions string - no API
                                        %exists to do this in OpenSim 4.0
                                        fid  = fopen(newModelFilename,'r');
                                        f=fread(fid,'*char')';
                                        fclose(fid);
                                        indexStartOfWO = strfind(f, ['name="' char(wrapObjName)]);
                                        indexDimensionString_0 = strfind(f(indexStartOfWO:end), '<dimensions>');
                                        indexDimensionString_1 = strfind(f(indexStartOfWO:end), '</dimensions>');
                                        indexDimensionString_0 = indexStartOfWO + indexDimensionString_0(1);
                                        indexDimensionString_1 = indexStartOfWO + indexDimensionString_1(1);
                                        newF = [f(1 : indexDimensionString_0 + 10) newDimensionString f(indexDimensionString_1 - 2 : end)];
                                        fid  = fopen(newModelFilename,'w');
                                        fprintf(fid,'%s',newF);
                                        fclose(fid);
                                        madeModifications = 1;
                                        wrapEllipsoidsModified{end+1} = char(wrapObjName);
                                        wrapEllipsoidsOrigDimension{end+1}= dimensionsString;
                                        wrapEllipsoidsModifiedDimension{end+1} = newDimensionString;
                                        break;
                                    else
                                        disp([muscleName ' moment arm wrong but wrap object is already too small! Check manually']);
                                        failed = 1;
                                        momentArmsAreWrong = 0;
                                    end
                                end
                            end
                        end
                    end
                end
            end

            if madeModifications == 1
                reCheckMotionFileNames = [];
                noDiscMotionFileNames = [];
                for u = 1 : numel(discontinuities)
                    if size(discontinuities{u}, 1) > 0
                        reCheckMotionFileNames{end+1} = motionFileNames{u};
                    end
                end
                noDiscMotionFileNames = motionFileNames(~ismember(motionFileNames, reCheckMotionFileNames));
                % check only the motion that had discontinuities first
                [momentArmsAreWrong, ~, discontinuities, ~, ~] = calcMuscleMomentArmsForMotion(newModelFilename, ...
                    reCheckMotionFileNames, coordinateNames, muscleFilter, threshold, 0, 1, filterFrequency);

                if ~momentArmsAreWrong
                    % if this file is good - check rest if nothing
                    % happened due to previous changes
                    [momentArmsAreWrong, ~, discontinuities, ~, ~] = calcMuscleMomentArmsForMotion(newModelFilename, ...
                        noDiscMotionFileNames, coordinateNames, muscleFilter, threshold, 0, 1, filterFrequency);
                end
            else
                momentArmsAreWrong = 0;
                fprintf(2, 'Some moment arms are wrong but it cannot be corrected automatically. Check manually');
                failed = 1;
            end
        else
            momentArmsAreWrong = 0;
        end
    end

    if failed ~= 1
        uniqueWrapObjects = unique(wrapObjectsModified);
        uniqueWrapEllipsoids = unique(wrapEllipsoidsModified);
        disp(' ');
        disp('SUMMARY: ');
        if numel(uniqueWrapObjects) > 0
            for m = 1 : numel(uniqueWrapObjects)
                indizes = find(contains(wrapObjectsModified, uniqueWrapObjects{m}));
                disp([uniqueWrapObjects{m} ' radius reduced from ' num2str(wrapObjectsOrigRadius(indizes(1))) ' to ' num2str(wrapObjectsModifiedRadius(indizes(end)))])
            end
        end
        if numel(uniqueWrapEllipsoids) > 0
            for m = 1 : numel(uniqueWrapEllipsoids)
                indizes = find(contains(wrapEllipsoidsModified, uniqueWrapEllipsoids{m}));
                disp([uniqueWrapEllipsoids{m} ' dimensions reduced from ' char(wrapEllipsoidsOrigDimension{indizes(1)}) ' to ' char(wrapEllipsoidsModifiedDimension{indizes(end)}) ])
            end
        end
        disp(['Model with modified wrapping object saved as ' newModelFilename]);
    else
        fprintf(2, '------------------------------------------------------------------------------');
        fprintf(2, 'Something went wrong - probably the optimization aborted after max. iterations');
        fprintf(2, '------------------------------------------------------------------------------');
    end
end
diary off;

FigList = findobj(allchild(0), 'flat', 'Type', 'figure');
savefig(FigList, figFilename);
close all;