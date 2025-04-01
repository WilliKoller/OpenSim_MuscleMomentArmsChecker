# OpenSim Muscle Moment Arms Checker

## Overview

The **OpenSim Muscle Moment Arms Checker** is a MATLAB tool designed to detect and resolve discontinuities in muscle moment arms and muscle length within musculoskeletal OpenSim models. This tool ensures physiologically plausible muscle paths by automatically adjusting Wrap Object radii in cases where muscle paths become discontinuous due to model scaling or anatomical variations.

## Features

- **Automated detection** of muscle moment arm discontinuities
- **Correction mechanism** that adjusts Wrap Object radii incrementally
- **Verified on a large dataset** (940 patients, 1,536 gait analysis sessions, showed more that 1600 discontinuities and only 17 could not be resolved automatically)

## Installation

### Requirements

To use this tool, ensure you have:

- MATLAB (latest version recommended)
- OpenSim (version 4.0 or newer)
- A compatible musculoskeletal model (e.g., Rajagopal model)

### Setup

1. Clone this repository:
   ```sh
   git clone https://github.com/WilliKoller/OpenSim_MuscleMomentArmsChecker.git
   ```
2. Add the repository folder to your MATLAB path.
   ```matlab
   addpath(genpath('path_to_repository'))
   ```
3. Ensure OpenSim is installed and properly configured for MATLAB.

## Usage

### Running the Script

1. **Load an OpenSim model** and prepare motion capture data.
2. **Open the MATLAB script**:
   ```matlab
   modifyWrappingObjectsToCorrectMuscleMomentArms.m
   ```
3. **Set your desired options**:

   ```matlab
   % approximately line 17
   stepSize = 0.001; % value which is used to shrink wrap objects each iteration
   threshold = 0.0028; % value that defines the threshold for discontinuity detection+
   filterFrequency = 6; % filter kinematic from .mot files; use -1 to disable

   kinematicFolder = './ExampleData\kinematics';
   kinematicFileFilter = '*.mot';
   modelFilename = './ExampleData\model.osim';
   ```

   ```matlab
   % approximately line 40
   % set coordinates and muscles that you want to check according to your model and movement
   coordinateNames = {'hip_flexion_l', 'hip_rotation_l', 'hip_adduction_l', ...
     'hip_flexion_r', 'hip_rotation_r', 'hip_adduction_r', ...
     'knee_angle_l', 'knee_angle_r'};

   % muscle that contain one of these texts will be checked by the script
   muscleFilter = {'add', 'gl', 'semi', 'bf', 'pec', 'grac', 'piri', 'sar', ...
    'tfl', 'iliacus', 'psoas', 'rect', 'gas', 'quad_fem', 'gem', 'peri', 'vas'};
   ```

4. The script will:
   - Detect discontinuities in muscle moment arm waveforms
   - Adjust the Wrap Object radius automatically
   - Output a corrected musculoskeletal model named like the original model with "\_modWO" appended.
   - Output a log file ("\_modifyWrapObjects.log") that contains the steps taken during the workflow
   - Run time depends on the computer specifications, numbers of coordinates, muscles, recording frequency and amount of files to check. A single step in walking should run in under 30 seconds. Example runs for several minutes, each motion contains several steps and it takes 6 iterations to resolve the discontinuities.

## Workflow

1. **Load Model & Data**: The tool loads the subject-specific OpenSim model and motion capture data.
2. **Check Muscle Moment Arms**: It evaluates muscle paths across the provided kinematics (e.g. gait cycle).
3. **Detect Discontinuities**: If abrupt changes in moment arms are found, the tool identifies the affected muscles.
4. **Correct Discontinuities**: The script modifies the radius of Wrap Objects in 1mm increments until smooth moment arms are achieved (max. 10 iterations = 1cm).
5. **Export Corrected Model**: The updated model with corrected muscle paths is saved for further simulations.
6. **Usage of Corrected Model**: This updated model should be used for all further simulations that involve muscles, e.g. Static Optimization.

## Citation

If you use this tool in your research, please cite:

> Koller, W., Horsak, B., Kranzl, A., Unglaube, F., Baca, A., & Kainz, H. (2025). Physiological plausible muscle paths: A MATLAB tool for detecting and resolving muscle path discontinuities in musculoskeletal OpenSim models. _Gait & Posture, 117_, S21-S22. https://doi.org/10.1016/j.gaitpost.2025.01.063

# License

<a rel="license" href="http://creativecommons.org/licenses/by-nc/4.0/" target="_blank"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" target="_blank" href="http://creativecommons.org/licenses/by-nc/4.0/">Creative Commons Attribution-NonCommercial 4.0 International License</a>.
Uses beyond those permitted by the license must be discussed with the authors.

## Contact

For questions or feedback, please contact the lead developer:

- **Willi Koller** (University of Vienna, Department of Biomechanics)  
  Email: [willi.koller@univie.ac.at](mailto:willi.koller@univie.ac.at)

---

This tool enhances the accuracy of musculoskeletal simulations by addressing muscle path discontinuities. We welcome contributions and feedback to improve its functionality!
