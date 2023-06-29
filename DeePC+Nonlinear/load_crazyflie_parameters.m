function param = load_crazyflie_parameters()

%% DEEPC PARAMETERS %%

param.drag = false;

% Flag to measure roll and pitch or only use position measurements in DeePC
param.measure_angles = false;

% Flag enables/disables DeePC yaw control
param.DeePC_yaw_control = false;

% Flag to plot predictions or not
param.plot_predictions = false;

% Flag to use real experimental data for DeePC
param.use_real_data = false;

% Solver to use for yalmip
param.yalmip_solver = 'osqp';

% Flag to make solver verbose (print its output)
param.verbose = false;

% Optimizer selector, default is yalmip
param.optimizer = 'osqp';

% Flag to setup optimization in sparse formulation
param.setup_sparse_opt = true;

% Flag to optimizer over steady state gs and us
param.opt_steady_state = false;

% Flag to perform two stage optimization on yini slack - POSSIBLE WITH
% YALMIP ONLY
% With this we solve for smallest feasible slack
% variable on yini first, then feed 'slacked' yini into DeePC optimization
% This eliminates slack tuning parameter
param.two_stage_opt = false;

% Flag to compare to linear noise free data. When set to true optimization
% is performed twice in each time-step for comparison
param.compare_to_perf = false;

% Flag to perform SysID on data
param.use_sysID = false;

%% --------------------------------------------------------------------- %%
%  Modelling, Control, and Estimator of N-rotor vehicles
%
%% CONTROL ARCHITECTURE AND TUNING OF PID AND LQR CONTROLLERS
%  
%  Note:
%  PID stands for Proportional Integral Derivative
%  LQR stands for Linear Quadratic Regulator
%
%% --------------------------------------------------------------------- %%


%% ASSUMPTIONS:
%  1) the origin of the body frame is at the center of mass of the N-rotor
%     vehicle.
%  2) The thrusts from the propellers are all aligned with the positive
%     z-axis of the body frame.
%  3) Gravity is aligned with the negative z-axis of the inertial frame


%  ---------------------------------------------------------------------  %
%   V   V  EEEEE  H   H  III   CCCC  L      EEEEE
%   V   V  E      H   H   I   C      L      E
%   V   V  EEE    HHHHH   I   C      L      EEE
%    V V   E      H   H   I   C      L      E
%     V    EEEEE  H   H  III   CCCC  LLLLL  EEEEE
%
%   PPPP     A    RRRR     A    M   M  EEEEE  TTTTT  EEEEE  RRRR    SSSS
%   P   P   A A   R   R   A A   MM MM  E        T    E      R   R  S
%   PPPP   A   A  RRRR   A   A  M M M  EEE      T    EEE    RRRR    SSS
%   P      AAAAA  R  R   AAAAA  M   M  E        T    E      R  R       S
%   P      A   A  R   R  A   A  M   M  EEEEE    T    EEEEE  R   R  SSSS
%  ---------------------------------------------------------------------  %
%% PARAMETERS OF THE N-ROTOR VEHICLE

% Gravitational constant:
param.g = 9.81;


%% --------------------------------------------------------------------- %%
%% STUDENTS TO SPECIFY LAYOUT AND PARAMETERS OF THEIR N-ROTOR DESIGN HERE:

% The number of rotors
param.nu = 4;

% The TRUE mass of the N-rotor vehicle, used for simulating the equations
% of motion [kg]

% % Crazyflie
% param.nrotor_vehicle_mass_true = 28e-3;

% Iris+
param.nrotor_vehicle_mass_true = 1.37;
param.payload = 0.4/param.nu;


% The "measured" mass of the N-rotor vehicle, USED FOR computing the
% FEED-FORWARD THRUST and CONTROLLER GAINS [kg]
% param.nrotor_vehicle_mass_for_controller = 28e-3;
param.nrotor_vehicle_mass_for_controller = param.nrotor_vehicle_mass_true;

% Mass moment of inertial of the N-rotor vehicle [kg m^2]

% % Crazyflie
% param.nrotor_vehicle_inertia = [...
%         16.57 ,   0.83 ,   0.72 ;...
%          0.83 ,  16.66 ,   1.80 ;...
%          0.72 ,   1.80 ,  29.26  ...
%     ] * 1e-6;

% Iris+
% param.nrotor_vehicle_inertia = [...
%         0.0238 ,   0 ,   0 ;...
%          0 ,  0.00882 ,   0 ;...
%          0 ,   0 ,  0.0303  ...
%     ];

param.nrotor_vehicle_inertia = [...
        0.0219 ,   0 ,   0 ;...
         0 ,  0.0109 ,   0 ;...
         0 ,   0 ,  0.0306  ...
    ];

% Inverse of the moment of inertia for convenience
param.nrotor_vehicle_inertia_inverse = inv(param.nrotor_vehicle_inertia);


% The the layout of the N-rotor vehicle:
% NOTE: the layout is specified as a matrix with the following format:
% nrotor_vehicle_layout = [...
%       x_1  ,  x_2  ,  ...  ,  x_{N-1}  ,  x_{N}  ;...
%       y_1  ,  y_2  ,  ...  ,  y_{N-1}  ,  y_{N}  ;...
%       c_1  ,  c_2  ,  ...  ,  c_{N-1}  ,  c_{N}   ...
%   ];
% where:
%   > x_i   is the x coordinate of propeller "i" relative to the CoG
%   > y_i   is the y coordinate of propeller "i" relative to the CoG
%   > c_i   is the thurst to torque coefficient of the propeller, with the
%           sign used to indicate the propeller's direction of rotation,
%           +ve indicates clock-wise propeller rotation.


% Specify a "baseline" value for the (x_i,y_i) layout
% % Crazyflie
% xi_baseline = (0.092/2) / sqrt(2);
% yi_baseline = (0.092/2) / sqrt(2);

% Iris+
xi_baseline = (0.0923 + 0.13)/2;
yi_baseline = (0.2537 + 0.2252)/2;


% Specify a "baseline" value for the "torque-to-thrust" ratio to make
% constructing the "nrotor_vehicle_layout" matrix cleaner

% % Crazyflie
% ci_baseline = 0.00596;

% Iris+
C_T = (0.1286+0.1059)/2;
C_P = (0.0431+0.0531)/2;
C_Q = C_P/(2*pi);
ci_baseline = C_Q/C_T;
% ci_baseline = 0.6362;

% The true symmetric Quad-rotor layout in "X" formation, used for
% simulating the equations of motion
param.nrotor_vehicle_layout_true = [...
        [ -1.0  ,  -1.0  ,  1.0 ,  1.0  ] * xi_baseline + 0.000  ;...
        [1.0  ,  -1.0  ,   -1.0 ,  1.0  ] * yi_baseline + 0.000  ;...
        [1    ,   -1    ,  1   ,  -1    ] * ci_baseline   ...
    ];


% SPECIFY THE MAXIMUM AND MINIMUM THRUST THAT CAN BE PRODUCE BY EACH
% PROPELLER:
% > This should be in unit of [Newtons]

% % Crazyflie
% % > For the minimum thrust: (this should always be zero)
% param.nrotor_vehicle_thrust_min = zeros(size(param.nrotor_vehicle_layout_true, 2), 1);
% % > For the maximum thrust:
% param.nrotor_vehicle_thrust_max = 0.1597*ones(size(param.nrotor_vehicle_layout_true, 2), 1);

% Iris+
% > For the minimum thrust: (this should always be zero)
param.nrotor_vehicle_thrust_min = zeros(size(param.nrotor_vehicle_layout_true, 2), 1);
% > For the maximum thrust:
param.nrotor_vehicle_thrust_max = (param.payload+param.nrotor_vehicle_mass_true)...
    *param.g*ones(size(param.nrotor_vehicle_layout_true, 2), 1);


% SPECIFY THE DC MOTOR LOW PASS FILTER TIME CONSTANT:
% > This should be in unit of [seconds]
param.nrotor_tau_M = 0.01;

% SPECIFY FRICTION MATRICES

if param.drag
    param.nrotor_vehicle_position_friction = diag([1, 1, 2]);
else
    param.nrotor_vehicle_position_friction = diag([0, 0, 0]);
end

param.nrotor_vehicle_body_rates_friction = diag([0, 0, 0]);

%% END OF SECTION WHERE STUDENTS SPECIFY THEIR N-ROTOR DESIGN
%% --------------------------------------------------------------------- %%
%   III  N   N  III  TTTTT  III    A    L
%    I   NN  N   I     T     I    A A   L
%    I   N N N   I     T     I   A   A  L
%    I   N  NN   I     T     I   AAAAA  L
%   III  N   N  III    T    III  A   A  LLLL
%
%    CCCC   OOO   N   N  DDDD   III  TTTTT  III   OOO   N   N   SSSS
%   C      O   O  NN  N  D   D   I     T     I   O   O  NN  N  S
%   C      O   O  N N N  D   D   I     T     I   O   O  N N N   SSS
%   C      O   O  N  NN  D   D   I     T     I   O   O  N  NN      S
%    CCCC   OOO   N   N  DDDD   III    T    III   OOO   N   N  SSSS
%  ---------------------------------------------------------------------  %
%% INITIAL CONDITIONS FOR THE SIMULINK MODEL

% Initial conditions for the full state
%nrotor_initial_condition_p       =    [-0.3; -0.5; 0.3];
nrotor_initial_condition_p       =    [0; 0; 0];
nrotor_initial_condition_p_dot   =    [0; 0; 0];
nrotor_initial_condition_psi     =    [0; 0; 0];
nrotor_initial_condition_psi_dot =    [0; 0; 0];
param.nrotor_initial_condition = [...
        nrotor_initial_condition_p          ;...
        nrotor_initial_condition_p_dot      ;...
        nrotor_initial_condition_psi        ;...
        nrotor_initial_condition_psi_dot    ;...
    ];





%% --------------------------------------------------------------------- %%
%   FFFFF  RRRR   EEEEE   QQQ   U   U  EEEEE  N   N   CCCC  Y   Y      /
%   F      R   R  E      Q   Q  U   U  E      NN  N  C       Y Y      /
%   FFF    RRRR   EEE    Q   Q  U   U  EEE    N N N  C        Y      /
%   F      R  R   E      Q  Q   U   U  E      N  NN  C        Y     /
%   F      R   R  EEEEE   QQ Q   UUU   EEEEE  N   N   CCCC    Y    /
%
%   SSSS    A    M   M  PPPP   L      EEEEE       TTTTT  III  M   M  EEEEE
%  S       A A   MM MM  P   P  L      E             T     I   MM MM  E
%   SSS   A   A  M M M  PPPP   L      EEE           T     I   M M M  EEE
%      S  AAAAA  M   M  P      L      E             T     I   M   M  E
%  SSSS   A   A  M   M  P      LLLLL  EEEEE         T    III  M   M  EEEEE
%  ---------------------------------------------------------------------  %
%% SPECIFY THE FREQUENCY OF THE MEASUREMENTS AND CONTROLLER

% For convenience of Simulink, the specifications here are in terms of the
% sample time, i.e., the recipricol of the frequency

% Sample time for the different types of measurements:
% > for the full state measurement
param.sample_time_measurements_full_state = 1 / 25;
% > for the body rates measurement taken by the on-board gyroscope
param.sample_time_measurements_body_rates = 1/500;
% > for the body accelerations measurement taken by the on-board
%   accelerometer
param.sample_time_measurements_body_accelerations = param.sample_time_measurements_body_rates;

% Sample time for the OUTER loop controller
param.sample_time_controller_outer = param.sample_time_measurements_full_state;
% Sample time for the INNER loop controller
param.sample_time_controller_inner = param.sample_time_measurements_body_rates;

param.exc_sample_time = param.sample_time_controller_outer;





%  ---------------------------------------------------------------------  %
%    CCCC   OOO   N   N  TTTTT  RRRR    OOO   L      L      EEEEE  RRRR
%   C      O   O  NN  N    T    R   R  O   O  L      L      E      R   R
%   C      O   O  N N N    T    RRRR   O   O  L      L      EEE    RRRR
%   C      O   O  N  NN    T    R  R   O   O  L      L      E      R  R
%    CCCC   OOO   N   N    T    R   R   OOO   LLLLL  LLLLL  EEEEE  R   R
%  ---------------------------------------------------------------------  %
%% STUDENTS TO ADD CONTROLLER PARAMETERS AND COMPUTATION HERE:
% Linearized model
% param.Ac = [zeros(3) eye(3) zeros(3);...
%      0 0 0 -param.nrotor_vehicle_position_friction(1,:) 0 -param.g 0;...
%      0 0 0 -param.nrotor_vehicle_position_friction(2,:) param.g 0 0;...
%      0 0 0 -param.nrotor_vehicle_position_friction(3,:) 0 0 0;...
%      zeros(3,9)];
param.Ac = [zeros(3) eye(3) zeros(3);...
     0 0 0 zeros(1,3) 0 -param.g 0;...
     0 0 0 zeros(1,3) param.g 0 0;...
     0 0 0 zeros(1,3) 0 0 0;...
     zeros(3,9)];
param.Bc = [zeros(5,4);...
    -1/param.nrotor_vehicle_mass_true 0 0 0;...
    zeros(3,1) eye(3)];

if param.measure_angles
    Cc = [eye(3) zeros(3, 6); zeros(3, 6) eye(3)];
else
    Cc = [eye(3) zeros(3, 6); zeros(1, 8) 1];
end

Dc = zeros(size(Cc,1),size(param.Bc,2));

sysC = ss(param.Ac, param.Bc, Cc, Dc);
sysD = c2d(sysC, param.sample_time_controller_outer);

param.Ad = sysD.A;
param.Bd = sysD.B;
param.Cd = sysD.C;
param.Dd = sysD.D;

param.Ad_MPC = sysD.A;
param.Bd_MPC = sysD.B;
param.Cd_MPC = sysD.C;

% param.Q = diag([50, 50, 1000, 0, 0, 0, 0, 0, 40]);
% param.R = diag([100, 100, 100, 2]);

% param.Q = diag([50, 50, 500, 0, 0, 0, 0, 0, 40]);
% param.R = diag([1, 10, 10, 2]);

param.Q = diag([40, 40, 500, 0, 0, 0, 0, 0, 40]);
param.R = diag([0.5, 20, 20, 20]);


[param.K_lqr_outer_loop, param.P] = dlqr(param.Ad, param.Bd, param.Q, param.R);

%% END OF SECTION WHERE STUDENTS ADD CONTROLLER PARAMETERS AND COMPUTATION
%% --------------------------------------------------------------------- %%


%% ASCII ART OF THE CRAZYFLIE 2.0 LAYOUT
%
%  > M1 to M4 stand for Motor 1 to Motor 4
%  > "CW"  indicates that the motor rotates Clockwise
%  > "CCW" indicates that the motor rotates Counter-Clockwise
%
%
%        ____                         ____
%       /    \                       /    \
%  (CW) | M4 |           x           | M1 | (CCW)
%       \____/\          ^          /\____/
%            \ \         |         / /
%             \ \        |        / /
%              \ \______ | ______/ /
%               \        |        /
%                |       |       |
%        y <-------------o       |
%                |               |
%               / _______________ \
%              / /               \ \
%             / /                 \ \
%        ____/ /                   \ \____
%       /    \/                     \/    \
% (CCW) | M3 |                       | M2 | (CW)
%       \____/                       \____/
%
%
%
%  ---------------------------------------------------------------------  %





%% PID PARAMETERS - FOR THE INNER BODY RATES CONTROLLER
%  > Parameters are taken directly from the crazyflie firmware,
%  > The derivatives gain have been reduced by a factor of 10 for avoid
%    high-frequency switch of control actuation.

param.pid_x_body_rate_kp = 250.0;
param.pid_x_body_rate_ki = 500.0;
param.pid_x_body_rate_kd =   2.5*0.1;
param.pid_x_body_rate_integrator_limit = 33.3;

param.pid_y_body_rate_kp = 250.0;
param.pid_y_body_rate_ki = 500.0;
param.pid_y_body_rate_kd =   2.5*0.1;
param.pid_y_body_rate_integrator_limit = 33.3;

param.pid_z_body_rate_kp = 120.0;
param.pid_z_body_rate_ki =  16.7;
param.pid_z_body_rate_kd =   0.0;
param.pid_z_body_rate_integrator_limit = 167.7;




%  ---------------------------------------------------------------------  %
%   M   M  EEEE    A     SSSS  U  U  RRRR   EEEE  M   M  EEEE  N   N  TTTTT
%   MM MM  E      A A   S      U  U  R   R  E     MM MM  E     NN  N    T
%   M M M  EEE   A   A   SSS   U  U  RRRR   EEE   M M M  EEE   N N N    T
%   M   M  E     AAAAA      S  U  U  R  R   E     M   M  E     N  NN    T
%   M   M  EEEE  A   A  SSSS    UU   R   R  EEEE  M   M  EEEE  N   N    T
%
%   N   N   OOO   III   SSSS  EEEEE
%   NN  N  O   O   I   S      E
%   N N N  O   O   I    SSS   EEE
%   N  NN  O   O   I       S  E
%   N   N   OOO   III  SSSS   EEEEE
%  ---------------------------------------------------------------------  %
%% SPECIFY THE MEASUREMENT NOISE PARAMETERS

% Degrees to radians conversion
param.deg2rad = pi/180;
param.rad2deg = 1 / param.deg2rad;

% Specify which noise signals noise to include
% > 0 = don't include, 1 = include
param.measurement_noise_full_state = 1;
param.measurement_noise_body_rates = 1;
param.measurement_noise_body_accelerations = 1;



% Create a random number gerneator that will be used to generate seeds
measurement_noise_randStream_seed = 42;
measurement_noise_randStream_for_seeds = RandStream.create(...
        'mrg32k3a' ,...
        'NumStreams' , 1 ,...
        'Seed' , measurement_noise_randStream_seed ...        
    );



% MEAN AND VARIANCE SPECIFICATIONS:

% > For the poisition meaurement
%   (units of meters for the mean)
measurement_noise_p_mean    =  zeros(3,1);
measurement_noise_p_stddev  =  [ 0.0001 ; 0.0001 ; 0.0001  ];
%measurement_noise_p_stddev  =  [ 0.001 ; 0.001 ; 0.001  ];
measurement_noise_p_var     =  measurement_noise_p_stddev.^2;
% Different seeds for running and data collection
measurement_noise_p_seed_run     =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
measurement_noise_p_seed_data =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);

% > For the translation velocity meaurement
measurement_noise_p_dot_mean    =  zeros(3,1);
measurement_noise_p_dot_stddev  =  2 * measurement_noise_p_stddev;
measurement_noise_p_dot_var     = measurement_noise_p_dot_stddev.^2;
% Different seeds for running and data collection
measurement_noise_p_dot_seed_run     =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
measurement_noise_p_dot_seed_data =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);

% > For the euler anlge meaurement
measurement_noise_psi_mean    =  zeros(3,1);
measurement_noise_psi_stddev  =  [ 0.2 ; 0.2 ; 0.2  ] * param.deg2rad;
measurement_noise_psi_var     =  measurement_noise_psi_stddev.^2;
% Different seeds for running and data collection
measurement_noise_psi_seed_run     =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
measurement_noise_psi_seed_data =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);

% > For the euler anlgar velocity meaurement
measurement_noise_psi_dot_mean    =  zeros(3,1);
measurement_noise_psi_dot_stddev  =  2 * measurement_noise_psi_stddev;
measurement_noise_psi_dot_var     =  measurement_noise_psi_dot_stddev.^2;
% Different seeds for running and data collection
measurement_noise_psi_dot_seed_run     =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
measurement_noise_psi_dot_seed_data =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);


% > For the full state measurement stacked together
param.measurement_noise_full_state_mean = [...
        measurement_noise_p_mean        ;...
        measurement_noise_p_dot_mean    ;...
        measurement_noise_psi_mean      ;...
        measurement_noise_psi_dot_mean   ...
    ];
measurement_noise_full_state_var = [...
        measurement_noise_p_var        ;...
        measurement_noise_p_dot_var    ;...
        measurement_noise_psi_var      ;...
        measurement_noise_psi_dot_var   ...
    ];
param.measurement_noise_full_state_seed_run = [...
        measurement_noise_p_seed_run        ;...
        measurement_noise_p_dot_seed_run    ;...
        measurement_noise_psi_seed_run      ;...
        measurement_noise_psi_dot_seed_run   ...
    ];
param.measurement_noise_full_state_seed_data = [...
        measurement_noise_p_seed_data        ;...
        measurement_noise_p_dot_seed_data    ;...
        measurement_noise_psi_seed_data      ;...
        measurement_noise_psi_dot_seed_data   ...
    ];
% By default, set seed to running seed
param.measurement_noise_full_state_seed  =  param.measurement_noise_full_state_seed_run;


% > For the gyroscope meaurement of the body rates
param.measurement_noise_gyroscope_mean    =  zeros(3,1);
measurement_noise_gyroscope_stddev  =  [ 1.0 ; 1.0 ; 1.0 ] * param.deg2rad;
measurement_noise_gyroscope_var     =  measurement_noise_gyroscope_stddev.^2;
% Different seeds for running and data collection
param.measurement_noise_gyroscope_seed_run     =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
param.measurement_noise_gyroscope_seed_data =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
% By default, set seed to running seed
param.measurement_noise_gyroscope_seed  =  param.measurement_noise_gyroscope_seed_run;

% > For the accelerometer meaurement of the body frame accelerations
param.measurement_noise_accelerometer_mean    =  zeros(3,1);
measurement_noise_accelerometer_stddev  =  [ 1.0 ; 1.0 ; 1.0 ];
measurement_noise_accelerometer_var     =  measurement_noise_accelerometer_stddev.^2;
% Different seeds for running and data collection
param.measurement_noise_accelerometer_seed_run     =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
param.measurement_noise_accelerometer_seed_data =  randi(measurement_noise_randStream_for_seeds,2^32-1,3,1);
% By default, set seed to running seed
param.measurement_noise_accelerometer_seed  =  param.measurement_noise_accelerometer_seed_run;



% % MEAN AND CO-VARIANCE SPECIFICATIONS:

% > For the full state mean vector
%   - this is the "measurement_noise_full_state_mean" defined above

% > For the full state covariance matrix
%   - Specify a diagonal matrix of the variances as the baseline
measurement_noise_full_state_covariance_matrix = diag( measurement_noise_full_state_var );
%   - Add some small covariance between the x and y position measurements
measurement_noise_full_state_covariance_matrix(1,2) = measurement_noise_full_state_var(1)/2;
measurement_noise_full_state_covariance_matrix(2,1) = measurement_noise_full_state_var(1)/2;
%   - Compute the decomposition needed for computing a multi-variate
%     Gaussian sample given a sample from a standard normal distribution
[U , D] = eig( full( measurement_noise_full_state_covariance_matrix ) );
param.measurement_noise_full_state_covariance_decomposition = U * sqrt(D);


% > For the gyroscope mean vector
%   - this is the "measurement_noise_gyroscope_mean" defined above

% > For the gyroscope covariance matrix
%   - Specify a diagonal matrix of the variances as the baseline
measurement_noise_gyroscope_covariance_matrix = diag(measurement_noise_gyroscope_var );
%   - Compute the decomposition needed for computing a multi-variate
%     Gaussian sample given a sample from a standard normal distribution
[U , D] = eig( full( measurement_noise_gyroscope_covariance_matrix ) );
param.measurement_noise_gyroscope_covariance_decomposition = U * sqrt(D);


% > For the accelerometer mean vector
%   - this is the "measurement_noise_accelerometer_mean" defined above

% > For the gyroscope covariance matrix
%   - Specify a diagonal matrix of the variances as the baseline
measurement_noise_accelerometer_covariance_matrix = diag( measurement_noise_accelerometer_var );
%   - Compute the decomposition needed for computing a multi-variate
%     Gaussian sample given a sample from a standard normal distribution
[U , D] = eig( full( measurement_noise_accelerometer_covariance_matrix ) );
param.measurement_noise_accelerometer_covariance_decomposition = U * sqrt(D);



%  ---------------------------------------------------------------------  %
%                      t     1    6           CCCC  M   M  DDDD
%   u   u  i    n nn  ttt   11   6           C      MM MM  D   D
%   u   u  i    nn  n  t     1   6666        C      M M M  D   D
%   u  uu  i    n   n  t     1   6   6       C      M   M  D   D
%    uu u  iii  n   n  ttt  111   666         CCCC  M   M  DDDD
%  ---------------------------------------------------------------------  %
%% SPECIFY THE CONVERSION FROM [uint16] COMMAND TO THRUST

% Specify the coefficients of the conversion from a uint16 binary command
% to the propeller thrust in Newtons
param.cmd_2_newtons_conversion_quadratic_coefficient  =  1.3385e-10;
param.cmd_2_newtons_conversion_linear_coefficient     =  6.4870e-6;

% Specify that max and min for the uint16 binary command
param.cmd_max = 2^16-1;
param.cmd_min = 0;

%% DEEPC YAW CONTROL
% Flag enables/disables DeePC yaw control
% If DeePC yaw control is disabled, remove yaw subsystem from equation of
% motion, as well as yaw body rate.
% Yaw is then controlled with LQR controller not DeePC
if param.DeePC_yaw_control
    Cd = param.Cd;
else
    param.Ad(end,:) = [];
    param.Ad(:,end) = [];
    param.Bd(end,:) = [];
    param.Bd(:,end) = [];
    param.Cd(end,:) = [];
    % Don't delete last column of param.Cd as it is used to multiply by
    % full state
    Cd = param.Cd;
    Cd(:,end) = [];
    param.Dd(end,:) = [];
    param.Dd(end,:) = [];
end

%% --------------------------------------------------------------------- %%
%   DDDD   IIIII   SSSS  TTTTT  U   U  RRRR   BBBB     A    N   N   CCCC
%   D   D    I    S        T    U   U  R   R  B   B   A A   NN  N  C
%   D   D    I     SSS     T    U   U  RRRR   BBBB   A   A  N N N  C
%   D   D    I        S    T    U   U  R  R   B   B  AAAAA  N  NN  C
%   DDDD   IIIII  SSSS     T     UUU   R   R  BBBB   A   A  N   N   CCCC
%  ---------------------------------------------------------------------  %
%% TREAT GRAVITY AS CONSTANT DISTURBANCE AND AUGMENT IT TO STATE
param.n = size(param.Ad, 1); % n_state
param.m = size(param.Bd, 2); % n_input
param.p = size(param.Cd, 1); % n_output


dist_states = 6; % Gravity disturbs derivative of velocity in z
%dist_states = [];
param.d = length(dist_states);
dist_vec = zeros(param.n, param.d);
for i = 1 : param.d
    dist_vec(dist_states(i),i) = 1;
end
param.Adist = [param.Ad dist_vec; zeros(param.d, param.n + param.d)];
Bdist = [param.Bd; zeros(param.d, param.m)];
param.Cdist = [Cd zeros(param.p, param.d)];
Ddist = param.Dd;

%% INPUT/OUTPUT BOUNDS %%
%% Input constraints
thrust_min = 0.25 * sum(param.nrotor_vehicle_thrust_min);
%thrust_min = 0.25 * sum(param.nrotor_vehicle_thrust_max) - param.nrotor_vehicle_mass_for_controller * param.g;
thrust_max = 0.75 * sum(param.nrotor_vehicle_thrust_max);
%thrust_max = 0.75 * sum(param.nrotor_vehicle_thrust_max) - param.nrotor_vehicle_mass_for_controller * param.g;
body_rates_min = [-pi/2; -pi/2; -pi/2];
body_rates_max = [pi/2; pi/2; pi/2];
param.input_min = eye(param.m, 4) * [thrust_min; body_rates_min];
param.input_min(isnan(param.input_min)) = -inf;
param.input_max = eye(param.m, 4) * [thrust_max; body_rates_max];
param.input_max(isnan(param.input_max)) = inf;

%% Output constraints
position_min = [-4; -4; -4];
position_max = [4; 4; 4];
velocity_min = [-100; -100; -100];
velocity_max = [100; 100; 100];
angle_min = [-pi/6; -pi/6; -pi/6];
angle_max = [pi/6; pi/6; pi/6];


param.output_min = param.Cd * [position_min; velocity_min; angle_min];
param.output_min(isnan(param.output_min)) = -inf;
param.output_max = param.Cd * [position_max; velocity_max; angle_max];
param.output_max(isnan(param.output_max)) = inf;


fprintf("Done loading parameters\n\n");
end