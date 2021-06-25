with surrogate
idName= '13_8';
N0=100;
N_iter=50;
Nsample=10;
N_surrogate_repeat=10;
np2=2;

idName= '13_9';
N0=100;
N_iter=50;
Nsample=40;
N_surrogate_repeat=10;
np2=2;


idName= '13_10';
N0=200;
N_iter=50;
Nsample=40;
N_surrogate_repeat=10;
np2=2;

% initial values for GP of BO
idName= '13_11';
N0=10;
N_iter=50;
Nsample=40;
N_surrogate_repeat=10;
np2=2;

% initial values for GP of BO
idName= '13_12';
N0=10;
N_iter=50;
Nsample=5;
N_surrogate_repeat=10;
np2=2;

until here there was a bug, so results are not correct until here

% initial values for GP of BO
idName= '13_13';
N0=10;
N_iter=300;
Nsample=10;
N_surrogate_repeat=10;
np2=2;

% initial values for GP of BO
idName= '13_14';
N0=10;
N_iter=50;
Nsample=10;
N_surrogate_repeat=10;
np2=2;

% initial values for GP of BO
idName= '13_15';
N0=10;
N_iter=50;
Nsample=100;
N_surrogate_repeat=10;
np2=2;

% initial values for GP of BO
idName= '13_16';
N0=200;
N_iter=44;
Nsample=10;
N_surrogate_repeat=10;
np2=2;

% initial values for GP of BO
idName= '13_17';
N0=5;
N_iter=44;
Nsample=10;
N_surrogate_repeat=10;
np2=2;

% initial values for GP of BO
idName= '13_18';
N0=5;
N_iter=200;
Nsample=10;
N_surrogate_repeat=10;
np2=2;

% initial values for GP of BO
idName= '13_19';
N0=5;
N_iter=200;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% initial values for GP of BO
idName= '13_20';
N0=30;
N_iter=200;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% initial values for GP of BO
idName= '13_21';
N0=2;
N_iter=200;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% initial values for GP of BO
idName= '13_22';
N0=2;
N_iter=200;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% initial values for GP of BO
idName= '13_23';
N0=2;
N_iter=200;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=4;
np2=2;
-0.041191    0.16652    0.10413


%%%%%%%%%%%%%%%%%%%%%Bug corrected---previous surrogates were wrong
% hyper-params
idName= '13_24';
N0=2;
N_iter=100;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;


1++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
to show convergence


% hyper-params
idName= '14_1';
N0=10;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '14_2';
N0=10;
N_iter=50;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

2++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
to show Ms does better with lower N0

% hyper-params
idName= '14_3';
N0=3;
N_iter=50;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '14_4';
N0=3;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '14_5';
N0=20;
N_iter=50;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '14_6';
N0=20;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% robot-arm
1++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
to show convergence

% hyper-params
idName= '15_1';
N0=10;
N_iter=50;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '15_2';
N0=10;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;


2++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
to show Ms does better with lower N0

% hyper-params repeated 4 times
idName= '15_3';
N0=3;
N_iter=50;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params repeated 4 times
idName= '15_4';
N0=3;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '15_5';
N0=30;
N_iter=50;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '15_6';
N0=30;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;


4++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
to show increasing the number of data coming from surrogate helps

% hyper-params
idName= '15_14';
N0=10;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=5;
Nsample=10;
np2=2;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ball-screw
1++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
to show convergence

% hyper-params
idName= '15_7';
N0=10;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '15_8';
N0=10;
N_iter=50;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;


2++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
to show Ms does better with lower N0

% hyper-params 4 repeated
idName= '15_9';
N0=3;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params 4 repeated
idName= '15_10';
N0=3;
N_iter=50;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '15_11';
N0=30;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

% hyper-params
idName= '15_12';
N0=30;
N_iter=50;
withSurrogate=false;
N_surrogate_repeat=10;
Nsample=10;
np2=2;

4++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
to show increasing the number of data coming from surrogate helps


% hyper-params
idName= '15_13';
N0=10;
N_iter=50;
withSurrogate=true;
N_surrogate_repeat=5;
Nsample=10;
np2=2;




With Perturbed models


***repeat 3 times:


% hyper-params
idName= '16_1';
N0=10;
N_iter=50;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
num_perturbed_model=4;

% hyper-params
idName= '16_2';
N0=10;
N_iter=50;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
num_perturbed_model=4;

% hyper-params
idName= '16_3';
N0=10;
N_iter=50;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
num_perturbed_model=4;


***repeat 3 times:

% hyper-params
idName= '16_4';
N0=3;
N_iter=50;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
num_perturbed_model=4;

% hyper-params
idName= '16_5';
N0=3;
N_iter=50;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
num_perturbed_model=4;

% hyper-params
idName= '16_6';
N0=3;
N_iter=50;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
num_perturbed_model=4;


=================================================================================================================================
% hyper-params
idName= 'demo_17_1';
N0=10;
N_iter=60;
repeat_experiment=100;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=true;
num_perturbed_model=4;

% hyper-params
idName= 'demo_17_2';
N0=10;
N_iter=50;
repeat_experiment=100;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

****WRONG InitData so far!!!!!!!!!!!!!!!!!!!!!!!!!!
***Wrong Init data objective: not matching(not complete)

=================================================================================================================================
% hyper-params
idName= 'demo_19_1';
sys='robot_arm';
N0=50;
N_iter=50;
repeat_experiment=20;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

% hyper-params
idName= 'demo_19_2';
sys='robot_arm';
N0=50;
N_iter=50;
repeat_experiment=20;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=true;
num_perturbed_model=4;

% hyper-params
idName= 'demo_19_3';
sys='robot_arm';
N0=100;
N_iter=50;
repeat_experiment=20;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

% hyper-params
idName= 'demo_19_4';
sys='robot_arm';
N0=100;
N_iter=50+10;
repeat_experiment=20;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=true;
num_perturbed_model=4;


Correnctions for results applied
=================================================================================================================================
% hyper-params
idName= 'demo_20_1';
sys='robot_arm';
N0=20;
N_iter=50+10;
repeat_experiment=20;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=true;
num_perturbed_model=4;

% hyper-params
idName= 'demo_20_2';
sys='robot_arm';
N0=20;
N_iter=50;
repeat_experiment=20;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;

% hyper-params
idName= 'demo_20_3';
sys='robot_arm';
N0=10;
N_iter=50+10;
repeat_experiment=20;
withSurrogate=true;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=true;
num_perturbed_model=4;

% hyper-params
idName= 'demo_20_4';
sys='robot_arm';
N0=10;
N_iter=50;
repeat_experiment=20;
withSurrogate=false;
N_real_repeat=25;
Nsample=10;
np2=2;
withPerturbed=false;
num_perturbed_model=4;
