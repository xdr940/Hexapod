//====================================================================
// 这是YFRobot Hexapods Robot的硬件配置文件。
//
//  此版本的配置文件设置为在 YFRobot hexapods UNO板
//  此版本的配置文件假定伺服器将由YFRobot伺服控制器SSC-32控制，并且用户使用PS2来控制机器人。
//
//                                  \     /
//               Tibia Femur Coxa┌---------┐Coxa  Femur Tibia
//              LF   ----ο---ο--ο├         ┤ο--ο---ο----  RF
//                               ├         ┤
//              LM   ----ο---ο--ο├         ┤ο--ο---ο----  RM
//                               ├         ┤
//              LR   ----ο---ο--ο├         ┤ο--ο---ο----  RR
//                               └---------┘
//
//====================================================================
#ifndef HEX_CFG_H
#define HEX_CFG_H

//[CONDITIONAL COMPILING] - COMMENT IF NOT WANTED   #[条件编译] - 注释如果不想
// Define other optional compnents to be included or not...  #定义包含或不包含的其他可选组件
#define OPT_TERMINAL_MONITOR

#ifdef OPT_TERMINAL_MONITOR         // turning off terminal monitor will turn these off as well...  #关闭终端显示器也会关闭这些...
//#define OPT_SSC_FORWARDER           // only useful if terminal monitor is enabled  #仅在启用终端监视器时才有用  允许上位机软件通过arduino与SSC32通讯
#define OPT_FIND_SERVO_OFFSETS      // Only useful if terminal monitor is enabled  #仅在启用终端监视器时才有用  找舵机零点
#endif

//动作组模式未测试，需自行研究
//#define OPT_GPPLAYER                // 动作组模式定义参数

// Which type of control(s) do you want to compile in  #你想编译哪种类型的控件
#define DBGSerial         Serial
#if defined(UBRR1H)
#define SSCSerial         Serial1
#else
#endif

#define DEBUG_IOPINS            // LED显示状态
#define USEPS2                  // 定义使用PS2控制参数
#define USE_SSC32               // 定义使用SSC32控制板
//#define	cSSC_BINARYMODE	1			// Define if your SSC-32 card supports binary mode. #定义如果你的SSC-32卡支持二进制模式。

// Warning I will undefine some components as the non-megas don't have enough memory...
//  #警告我将取消定义一些组件，因为非megas没有足够的内存......
//#undef OPT_FIND_SERVO_OFFSETS

//[SERIAL CONNECTIONS #串口连接]
#define cSSC_BAUD        115200   //SSC32 BAUD rate #SSC32波特率

//--------------------------------------------------------------------
//[hexUNO PIN NUMBERS]
#define SOUND_PIN    5        // buzzer pin number
#define PS2_DAT      6        // PS2 pin number
#define PS2_CMD      7
#define PS2_SEL      8
#define PS2_CLK      9
// Use SoftwareSerial to talk to the SSC-32
#define cSSC_OUT     12      	//Output pin for (SSC32 RX)
#define cSSC_IN      13      	//Input pin for (SSC32 TX)

// Define Analog pin and minimum voltage that we will allow the servos to run
#define cVoltagePin  A0       // Use our Analog pin jumper here... 
#define cBuzzerAlarmVol  530  // 5.3v - 报警电压，当电压低于设定值时程序报警，电池需要充电

//Define OUTPUTS pin to show something...
#ifdef DEBUG_IOPINS
#define cEyesPin A1           // Servo - L3 yellow led
#define cBProcessPin A2       // Do Background Process - L2 yellow led
#define cIKErrorPin A3        // IK Error - L1 red led
#endif
//--------------------------------------------------------------------

//--------------------------------------------------------------------
//[SSC PIN NUMBERS]
#define cRRCoxaPin      0   //Rear Right leg Hip Horizontal   - 后 右 腿 水平关节
#define cRRFemurPin     1   //Rear Right leg Hip Vertical     - 后 右 腿 垂直关节
#define cRRTibiaPin     2   //Rear Right leg Knee            - 后 右 腿 膝盖
#define cRRTarsPin      3   // Tar                           - ? 4DOF

#define cRMCoxaPin      4   //Middle Right leg Hip Horizontal - 中 右 腿 水平关节
#define cRMFemurPin     5   //Middle Right leg Hip Vertical   - 中 右 腿 垂直关节
#define cRMTibiaPin     6   //Middle Right leg Knee           - 中 右 腿 膝盖
#define cRMTarsPin      7   // Tar                            - ? 4DOF

#define cRFCoxaPin      8   //Front Right leg Hip Horizontal  - 前 右 腿 水平关节
#define cRFFemurPin     9   //Front Right leg Hip Vertical    - 前 右 腿 垂直关节
#define cRFTibiaPin     10   //Front Right leg Knee            - 前 右 腿 膝盖
#define cRFTarsPin      11   // Tar                            - ? 4DOF

#define cLRCoxaPin      16   //Rear Left leg Hip Horizontal   - 后 左 腿 水平关节
#define cLRFemurPin     17   //Rear Left leg Hip Vertical     - 后 左 腿 垂直关节
#define cLRTibiaPin     18   //Rear Left leg Knee             - 后 左 腿 膝盖
#define cLRTarsPin      19   // Tar                           - ? 4DOF

#define cLMCoxaPin      20   //Middle Left leg Hip Horizontal - 中 左 腿 水平关节
#define cLMFemurPin     21   //Middle Left leg Hip Vertical   - 中 左 腿 垂直关节
#define cLMTibiaPin     22   //Middle Left leg Knee           - 中 左 腿 膝盖
#define cLMTarsPin      23   // Tar                           - ? 4DOF

#define cLFCoxaPin      24   //Front Left leg Hip Horizontal  - 前 左 腿 水平关节
#define cLFFemurPin     25   //Front Left leg Hip Vertical    - 前 左 腿 垂直关节
#define cLFTibiaPin     26   //Front Left leg Knee            - 前 左 腿 膝盖
#define cLFTarsPin      27   // Tar                           - ? 4DOF

//--------------------------------------------------------------------
//[Inverse Servo Direction - 反舵机方向]
#define cRRCoxaInv  1
#define cRMCoxaInv  1
#define cRFCoxaInv  1
#define cRRFemurInv 1
#define cRMFemurInv 1
#define cRFFemurInv 1
#define cRRTibiaInv 1
#define cRMTibiaInv 1
#define cRFTibiaInv 1

#define cLRCoxaInv  1
#define cLMCoxaInv  1
#define cLFCoxaInv  1
#define cLRFemurInv 1
#define cLMFemurInv 1
#define cLFFemurInv 1
#define cLRTibiaInv 1
#define cLMTibiaInv 1
#define cLFTibiaInv 1

//--------------------------------------------------------------------
//[MIN/MAX ANGLES  - 最小/最大 角度]
#define cRRCoxaMin1   -260
#define cRRCoxaMax1   740
#define cRRFemurMin1  -1010
#define cRRFemurMax1  950
#define cRRTibiaMin1  -1060
#define cRRTibiaMax1  770

#define cRMCoxaMin1   -530
#define cRMCoxaMax1   530
#define cRMFemurMin1  -1010
#define cRMFemurMax1  950
#define cRMTibiaMin1  -1060
#define cRMTibiaMax1  770

#define cRFCoxaMin1   -580
#define cRFCoxaMax1   740
#define cRFFemurMin1  -1010
#define cRFFemurMax1  950
#define cRFTibiaMin1  -1060
#define cRFTibiaMax1  770

#define cLRCoxaMin1   -740
#define cLRCoxaMax1   260
#define cLRFemurMin1  -950
#define cLRFemurMax1  1010
#define cLRTibiaMin1  -770
#define cLRTibiaMax1  1060

#define cLMCoxaMin1   -530
#define cLMCoxaMax1   530
#define cLMFemurMin1  -950
#define cLMFemurMax1  1010
#define cLMTibiaMin1  -770
#define cLMTibiaMax1  1060

#define cLFCoxaMin1   -740
#define cLFCoxaMax1   580
#define cLFFemurMin1  -950
#define cLFFemurMax1  1010
#define cLFTibiaMin1  -770
#define cLFTibiaMax1  1060

//--------------------------------------------------------------------
//[LEG DIMENSIONS - 腿 尺寸]
//Universal dimensions for each leg in mm  #每个腿的通用尺寸单位mm
#define cXXCoxaLength     29    // This is for TH3-R legs    
#define cXXFemurLength    85
#define cXXTibiaLength    125
#define cXXTarsLength     85    // 4DOF only...

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength
#define cRRTarsLength	    cXXTarsLength	    //4DOF ONLY

#define cRMCoxaLength     cXXCoxaLength	    //Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength
#define cRMTarsLength	    cXXTarsLength	    //4DOF ONLY

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength
#define cRFTarsLength	    cXXTarsLength    //4DOF ONLY

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength
#define cLRTarsLength	    cXXTarsLength    //4DOF ONLY

#define cLMCoxaLength     cXXCoxaLength	    //Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength
#define cLMTarsLength	    cXXTarsLength	    //4DOF ONLY

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength
#define cLFTarsLength	    cXXTarsLength	    //4DOF ONLY


//--------------------------------------------------------------------
//[BODY DIMENSIONS - 机体尺寸]
// Coxa 舵机角度设置
#define cRRCoxaAngle1   -600    //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    600    //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -600   //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    600    //Default Coxa setup angle, decimals = 1

// 机体尺寸单位mm
#define cRROffsetX      -40     //Distance X from center of the body to the Right Rear coxa    - 从机体中心到右后 距离X
#define cRROffsetZ      75      //Distance Z from center of the body to the Right Rear coxa    - 从机体中心到右后 距离Z
#define cRMOffsetX      -65     //Distance X from center of the body to the Right Middle coxa  - 从机体中心到右中 距离X
#define cRMOffsetZ      0       //Distance Z from center of the body to the Right Middle coxa  - 从机体中心到右中 距离Z
#define cRFOffsetX      -40     //Distance X from center of the body to the Right Front coxa   - 从机体中心到右前 距离X
#define cRFOffsetZ      -75     //Distance Z from center of the body to the Right Front coxa    - 从机体中心到右前 距离Z

#define cLROffsetX      40      //Distance X from center of the body to the Left Rear coxa    - 从机体中心到左后 距离X
#define cLROffsetZ      75      //Distance Z from center of the body to the Left Rear coxa    - 从机体中心到左后 距离Z
#define cLMOffsetX      65      //Distance X from center of the body to the Left Middle coxa  - 从机体中心到左中 距离X
#define cLMOffsetZ      0       //Distance Z from center of the body to the Left Middle coxa  - 从机体中心到左中 距离Z
#define cLFOffsetX      40      //Distance X from center of the body to the Left Front coxa   - 从机体中心到左前 距离X
#define cLFOffsetZ      -75     //Distance Z from center of the body to the Left Front coxa   - 从机体中心到左前 距离Z

//--------------------------------------------------------------------
//[START POSITIONS FEET - 起始位置]
#define cHexInitXZ       105
#define CHexInitXZCos60  53     // cos(60) = 0.5
#define CHexInitXZSin60  91     // sin(60) = 0.866
#define CHexInitY        25

// Lets try some multi leg positions depending on height settings. - 让我们根据高度设置尝试一些多腿的位置。
#define CNT_HEX_INITS 3
#define MAX_BODY_Y  150

#define DEFINE_HEX_GLOBALS
#ifdef DEFINE_HEX_GLOBALS
const byte g_abHexIntXZ[] PROGMEM = {cHexInitXZ, 99, 86};
const byte g_abHexMaxBodyY[] PROGMEM = {20, 50, MAX_BODY_Y};
#else
extern const byte g_abHexIntXZ[] PROGMEM;
extern const byte g_abHexMaxBodyY[] PROGMEM;
#endif

#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg    - 右后脚的开始位置
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ           //Start positions of the Right Middle leg  - 右中脚的开始位置
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg   - 右前脚的开始位置
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg     - 左后脚的开始位置
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ           //Start positions of the Left Middle leg   - 左中脚的开始位置
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg    - 左前脚的开始位置
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60

//--------------------------------------------------------------------
//[Tars factors used in formula to calc Tarsus angle relative to the ground 在公式中用于计算相对于地面的T角的焦油因子]
#define cTarsConst	720	  //4DOF ONLY
#define cTarsMulti	2	    //4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

#endif
