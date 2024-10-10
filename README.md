# OpenSim_MuscleMomentArmsChecker
This script checks muscle moment arms around joints for discontinuities and alters associated wrap objects to ensure smoothness and valid muscle paths.
1. open modifyWrappingObjectsToCorrectMuscleMomentArms.m
2. set your model (.osim file) and folder with model kinematics (i.e. results of inverse kinematics)
3. set coordinates and muscles according to your model 
4. run script (Depending on the degrees of freedom of the model, duration of the kinematics, computer specification this can run some time. Example files run in approximately 24 minutes on a normal notebook.)
5. check modification with compareMuscleParams.m
