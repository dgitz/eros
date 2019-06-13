%***************AUTO-GENERATED.  DO NOT EDIT********************/
%***Created on:%2019-06-13 08:15:36.984995%***/
global Message
Message.SUBSYSTEMDIAGNOSTIC_ID = 0XAB43;
Message.LEDSTRIPCONTROL_ID = 0XAB42;
Message.FIRMWARE_ID = 0XAB41;
Message.ID_ID = 0XAB40;
Message.TUNECONTROLGROUP_ID = 0XAB39;
Message.ESTOP_ID = 0XAB38;
Message.POWER_ID = 0XAB37;
Message.CONFIGURE_ANA_PORT_ID = 0XAB36;
Message.PPS_ID = 0XAB35;
Message.FINDTARGET_ID = 0XAB34;
Message.SET_DIO_PORT_DEFAULTVALUE_ID = 0XAB32;
Message.HEARTBEAT_ID = 0XAB31;
Message.ARM_STATUS_ID = 0XAB30;
Message.GET_IMUMAG_ID = 0XAB29;
Message.GET_IMUGYRO_ID = 0XAB28;
Message.GET_IMUACC_ID = 0XAB27;
Message.ARMCONTROL_ID = 0XAB26;
Message.FIRMWAREVERSION_ID = 0XAB25;
Message.GET_ANA_PORT1_ID = 0XAB20;
Message.GET_DIO_PORT1_ID = 0XAB19;
Message.SET_DIO_PORT_ID = 0XAB18;
Message.MODE_ID = 0XAB17;
Message.CONFIGURE_DIO_PORT_ID = 0XAB16;
Message.TESTMESSAGECOMMAND_ID = 0XAB15;
Message.TESTMESSAGECOUNTER_ID = 0XAB14;
Message.DEVICE_ID = 0XAB13;
Message.DIAGNOSTIC_ID = 0XAB12;
Message.RESOURCE_ID = 0XAB11;
Message.REMOTECONTROL_ID = 0XAB10;
Message.COMMAND_ID = 0XAB02;
Message.USERMESSAGE_ID = 0XAB01;


global System
System.ROVER = 1;
System.GROUND_STATION = 5;
System.REMOTE_CONTROL = 7;
System.SYSTEM_UNKNOWN = 255;


global Subsystem
Subsystem.ENTIRE_SYSTEM = 0;
Subsystem.ROBOT_CONTROLLER = 1;
Subsystem.ROBOT_MONITOR = 2;
Subsystem.SUBSYSTEM_UNKNOWN = 255;


global Component
Component.ENTIRE_SUBSYSTEM = 0;
Component.CONTROLLER_NODE = 1;
Component.DIAGNOSTIC_NODE = 2;
Component.NAVIGATION_NODE = 3;
Component.MAPPING_NODE = 7;
Component.EVOLUTION_NODE = 8;
Component.TARGETING_NODE = 9;
Component.TIMING_NODE = 10;
Component.VISION_NODE = 11;
Component.GPIO_NODE = 12;
Component.COMMUNICATION_NODE = 13;
Component.DYNAMICS_NODE = 14;
Component.POWER_NODE = 15;
Component.POSE_NODE = 16;
Component.COMPONENT_UNKNOWN = 255;


global DiagnosticType
DiagnosticType.ELECTRICAL = 1;
DiagnosticType.SOFTWARE = 2;
DiagnosticType.COMMUNICATIONS = 3;
DiagnosticType.SENSORS = 4;
DiagnosticType.ACTUATORS = 5;
DiagnosticType.DATA_STORAGE = 6;
DiagnosticType.REMOTE_CONTROL = 7;
DiagnosticType.TARGET_ACQUISITION = 8;
DiagnosticType.POSE = 9;
DiagnosticType.TIMING = 10;
DiagnosticType.SYSTEM_RESOURCE = 11;
DiagnosticType.GENERAL_ERROR = 255;


global DiagnosticLevel
DiagnosticLevel.DEBUG = 0;
DiagnosticLevel.INFO = 1;
DiagnosticLevel.NOTICE = 2;
DiagnosticLevel.WARN = 3;
DiagnosticLevel.ERROR = 4;
DiagnosticLevel.FATAL = 5;
DiagnosticLevel.LEVEL_UNKNOWN = 255;


global DiagnosticMessage
DiagnosticMessage.NOERROR = 0;
DiagnosticMessage.INITIALIZING = 1;
DiagnosticMessage.INITIALIZING_ERROR = 2;
DiagnosticMessage.DROPPING_PACKETS = 4;
DiagnosticMessage.MISSING_HEARTBEATS = 5;
DiagnosticMessage.DEVICE_NOT_AVAILABLE = 6;
DiagnosticMessage.ROVER_ARMED = 7;
DiagnosticMessage.ROVER_DISARMED = 8;
DiagnosticMessage.TEMPERATURE_HIGH = 9;
DiagnosticMessage.TEMPERATURE_LOW = 10;
DiagnosticMessage.DIAGNOSTIC_PASSED = 11;
DiagnosticMessage.DIAGNOSTIC_FAILED = 12;
DiagnosticMessage.RESOURCE_LEAK = 13;
DiagnosticMessage.HIGH_RESOURCE_USAGE = 14;
DiagnosticMessage.UNKNOWN_STATE = 15;
DiagnosticMessage.UNKNOWN_MESSAGE = 255;


global ArmedState
ArmedState.ARMEDSTATUS_UNDEFINED = 0;
ArmedState.ARMEDSTATUS_ARMED = 1;
ArmedState.ARMEDSTATUS_DISARMED_CANNOTARM = 2;
ArmedState.ARMEDSTATUS_DISARMED = 3;
ArmedState.ARMEDSTATUS_DISARMING = 4;
ArmedState.ARMEDSTATUS_ARMING = 5;


global SignalState
SignalState.SIGNALSTATE_UNDEFINED = 0;
SignalState.SIGNALSTATE_INVALID = 1;
SignalState.SIGNALSTATE_INITIALIZING = 2;
SignalState.SIGNALSTATE_UPDATED = 3;
SignalState.SIGNALSTATE_EXTRAPOLATED = 4;
SignalState.SIGNALSTATE_HOLD = 5;
SignalState.SIGNALSTATE_CALIBRATING = 6;




global SignalType
SignalType.SIGNALTYPE_UNDEFINED = 0;
SignalType.SIGNALTYPE_ACCELERATION = 1;
SignalType.SIGNALTYPE_ROTATION_RATE = 2;
SignalType.SIGNALTYPE_MAGNETIC_FIELD = 3;
SignalType.SIGNALTYPE_TEMPERATURE = 4;
SignalType.SIGNALTYPE_DISTANCE = 5;
SignalType.SIGNALTYPE_ANGLE = 6;


global LEDPixelMode
LEDPixelMode.LEDPIXELMODE_NONE = 0;
LEDPixelMode.LEDPIXELMODE_NORMAL = 1;
LEDPixelMode.LEDPIXELMODE_WARN = 2;
LEDPixelMode.LEDPIXELMODE_ERROR = 3;
LEDPixelMode.LEDPIXELMODE_COLORSELECT = 4;
LEDPixelMode.LEDPIXELMODE_FOCUS = 5;


global LEDPixelColor
LEDPixelColor.LEDPIXELCOLOR_OFF = 0;
LEDPixelColor.LEDPIXELCOLOR_RED = 1;
LEDPixelColor.LEDPIXELCOLOR_GREEN = 2;
LEDPixelColor.LEDPIXELCOLOR_BLUE = 3;
LEDPixelColor.LEDPIXELCOLOR_WHITE = 4;


