function one_DOF_s_function(block)
%MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
%   The MATLAB S-function is written as a MATLAB function with the
%   same name as the S-function. Replace 'msfuntmpl_basic' with the 
%   name of your S-function.

%   Copyright 2003-2018 The MathWorks, Inc.

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C MEX counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 2;

% Register parameters
block.NumDialogPrms     = 3;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
% block.InputPort(1).setName('tau');  
block.InputPort(1).Dimensions        =  1;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% block.InputPort(2).Name = 'h_e';  
% block.InputPort(2).Dimensions        =  6;
% block.InputPort(2).DatatypeID  = 0;  % double
% block.InputPort(2).Complexity  = 'Real';
% block.InputPort(2).DirectFeedthrough = true;


% Override output port properties
% block.OutputPort(1).Name = 'q';
block.OutputPort(1).Dimensions       =  1;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

% block.OutputPort(2).Name = 'dq';
block.OutputPort(2).Dimensions       =  1;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0];

block.NumContStates = 2;

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate);

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C MEX counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)

%end

%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C MEX counterpart: mdlInitializeConditions
%%
function InitializeConditions(block)

%end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C MEX counterpart: mdlStart
%%
function Start(block)

block.ContStates.Data(1) = block.DialogPrm(1).Data;
block.ContStates.Data(2) = block.DialogPrm(2).Data;

%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C MEX counterpart: mdlOutputs
%%
function Outputs(block)

block.OutputPort(1).Data = block.ContStates.Data(1);
block.OutputPort(2).Data = block.ContStates.Data(2);

% if block.CurrentTime == 0
%     % block.OutputPort(3).Data = [0 0 0]';
% else
%     block.OutputPort(3).Data = block.Derivatives.Data(block.DialogPrm(1).Data.DOF+1: block.DialogPrm(1).Data.DOF*2);
% end
% block.OutputPort(3).Data = block.Derivatives.Data(block.DialogPrm(1).Data.DOF+1: block.DialogPrm(1).Data.DOF*2);


%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlUpdate
%%
function Update(block)

% block.Dwork(1).Data = block.InputPort(1).Data;

%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlDerivatives
%%
function Derivatives(block)

% if block.CurrentTime == 0
%     block.ContStates.Data(1: block.DialogPrm(1).Data.DOF) = block.InputPort(2).Data;
%     block.ContStates.Data( block.DialogPrm(1).Data.DOF+1:  block.DialogPrm(1).Data.DOF*2) = block.InputPort(3).Data;
% end

q = block.ContStates.Data(1);
dq = block.ContStates.Data(2);
tau = block.InputPort(1).Data;


ddq = (1/block.DialogPrm(3).Data.I)  * (tau - block.DialogPrm(3).Data.F*dq - block.DialogPrm(3).Data.G * sin(q));

% block.Derivatives.Data(1: block.DialogPrm(1).Data.DOF) = dq;
% block.Derivatives.Data( block.DialogPrm(1).Data.DOF+1: block.DialogPrm(1).Data.DOF*2) = ddq;

block.Derivatives.Data = [dq;ddq];

% block.Dwork(1).Data = dq;
% block.Dwork(2).Data = ddq;

%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : No
%%   C MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate

