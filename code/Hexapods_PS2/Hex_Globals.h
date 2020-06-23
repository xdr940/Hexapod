//==============================================================================
// GLOBALS - The main global definitions for the CPhenix program - still needs
//		to be cleaned up.
// # GLOBALS - CPhenix程序的主要全局定义 - 仍需要清理。
// This program assumes that the main files were compiled as C files
// #该程序假定主文件被编译为C文件
//==============================================================================
#ifndef _HEX_GLOBALS_H_
#define _HEX_GLOBALS_H_
#include <stdarg.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>        // Beta 11 newsoftware serial...
#include "Hex_Cfg.h"

//=============================================================================
//[CONSTANTS] #常量
//=============================================================================
#define BUTTON_DOWN 0
#define BUTTON_UP 	1

#define	c1DEC		10
#define	c2DEC		100
#define	c4DEC		10000
#define	c6DEC		1000000

#define	cRR			0
#define	cRM			1
#define	cRF			2
#define	cLR			3
#define	cLM			4
#define	cLF			5

#define	WTIMERTICSPERMSMUL  	64	// BAP28 is 16mhz need a multiplyer and divider to make the conversion with /8192  #BAP28是16Mhz需要一个乘法器和分频器来进行 /8192 的转换
#define WTIMERTICSPERMSDIV  	125 // 
#define USEINT_TIMERAV

#define NUM_GAITS    6
#define SmDiv    4      //"Smooth division" factor for the smooth control function, a value of 3 to 5 is most suitable  #平滑控制功能的“平滑分割”因子，3到5的值是最合适的
extern void GaitSelect(void);
extern short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider);

//-----------------------------------------------------------------------------
// Define Global variables  #定义全局变量
//-----------------------------------------------------------------------------
extern boolean          g_fDebugOutput;
extern boolean          g_fEnableServos;       // Hack to allow me to turn servo processing off...  #允许我把伺服处理关闭
extern boolean          g_fRobotUpsideDown;    // Is the robot upside down?   #机器人是否倒置

extern void MSound(byte cNotes, ...);
extern boolean CheckVoltage(void);

void AdjustLegPositionsToBodyHeight(void);   //将腿部位置调整到身高

// debug handler...   #调试处理程序
extern boolean g_fDBGHandleError;

#ifdef c4DOF
extern const byte cTarsLength[] PROGMEM;
#endif

#ifdef OPT_BACKGROUND_PROCESS
#define DoBackgroundProcess()   g_ServoDriver.BackgroundProcess()
#else
#define DoBackgroundProcess()
#endif

#ifdef DEBUG_IOPINS
#define DebugToggle(pin)  {digitalWrite(pin, !digitalRead(pin));}
#define DebugWrite(pin, state) {digitalWrite(pin, state);}
#else
#define DebugToggle(pin)  {;}
#define DebugWrite(pin, state) {;}
#endif

#ifdef __AVR__
#if not defined(UBRR1H)
extern SoftwareSerial SSCSerial;
#endif
#endif
#if defined(__PIC32MX__)
#if defined F
#undef F
#endif
#define F(X) (X)
#endif

//=============================================================================
// Define the class(s) for our Input controllers.
//=============================================================================
class InputController {
  public:
    virtual void     Init(void);
    virtual void     ControlInput(void);
    virtual void     AllowControllerInterrupts(boolean fAllow);

  private:
} ;

// Define a function that allows us to define which controllers are to be used.
// #定义一个允许我们定义要使用哪些控制器的功能。
extern void  RegisterInputController(InputController *pic);

typedef struct _Coord3D {
  long      x;
  long      y;
  long      z;
} COORD3D;

//==============================================================================
// class ControlState: This is the main structure of data that the Control
//      manipulates and is used by the main Phoenix Code to make it do what is
//      requested.
//  这是Control操作的主要数据结构，并由Phoenix Code代码使用，以使其执行所请求的操作。
//==============================================================================
typedef struct _InControlState {
  boolean     fHexOn;              //Switch to turn on Phoenix
  boolean     fPrev_HexOn;         //Previous loop state
  //Body position
  COORD3D     BodyPos;
  COORD3D     BodyRotOffset;  // Body rotation offset;

  //Body Inverse Kinematics  #身体反向运动学
  COORD3D     BodyRot1;       // X -Pitch, Y-Rotation, Z-Roll

  //[gait]  #步态
  byte        GaitType;            //Gait type  #步态类型

  short       LegLiftHeight;       //Current Travel height
  COORD3D     TravelLength;   // X-Z or Length, Y is rotation.

  //[Single Leg Control]
  byte        SelectedLeg;
  COORD3D     SLLeg;          //
  boolean     fSLHold;             //Single leg control mode


  //[Balance]
  boolean     BalanceMode;

  //[TIMING]
  byte        InputTimeDelay;     //Delay that depends on the input to get the "sneaking" effect
  word        SpeedControl;       //Adjustible Delay
  byte        ForceGaitStepCnt;  // new to allow us to force a step even when not moving
} INCONTROLSTATE;

//==============================================================================
// Define the class(s) for Servo Drivers.
//==============================================================================
class ServoDriver {
  public:
    void Init(void);

    word GetBatteryVoltage(void);   // 获取电池电压

#ifdef OPT_GPPLAYER
    inline boolean  FIsGPEnabled(void) {
      return _fGPEnabled;
    };
    boolean         FIsGPSeqDefined(uint8_t iSeq);
    inline boolean  FIsGPSeqActive(void) {
      return _fGPActive;
    };
    void            GPStartSeq(uint8_t iSeq);         // 0xff - says to abort...
    void            GPPlayer(void);
    uint8_t         GPNumSteps(void);                 // How many steps does the current sequence have  #当前序列步数
    uint8_t         GPCurStep(void);                  // Return which step currently on...  #返回当前步
    void            GPSetSpeedMultiplyer(short sm) ;  // Set the Speed multiplier (100 is default)  #设置速度 （默认是100）
#endif
    void BeginServoUpdate(void);    // Start the update
#ifdef c4DOF
    void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1, short sTarsAngle1);
#else
    void OutputServoInfoForLeg(byte LegIndex, short sCoxaAngle1, short sFemurAngle1, short sTibiaAngle1);
#endif
    void CommitServoDriver(word wMoveTime);
    void FreeServos(void);

    // Allow for background process to happen...  #允许后台进程发生......
#ifdef OPT_BACKGROUND_PROCESS
    void BackgroundProcess(void);
#endif

#ifdef OPT_TERMINAL_MONITOR
    void ShowTerminalCommandList(void);
    boolean ProcessTerminalCommand(byte *psz, byte bLen);
#endif

  private:

#ifdef OPT_GPPLAYER
    boolean _fGPEnabled;     // IS GP defined for this servo driver?
    boolean _fGPActive;      // Is a sequence currently active - May change later when we integrate in sequence timing adjustment code
    uint8_t    _iSeq;        // current sequence we are running
    short    _sGPSM;         // Speed multiplier +-200
#endif

} ;

//-----------------------------------------------------------------------------
// Define global class objects   #定义全局类对象
//-----------------------------------------------------------------------------
extern ServoDriver      g_ServoDriver;           // our global servo driver class
extern InputController  g_InputController;       // Our Input controller
extern INCONTROLSTATE   g_InControlState;         // State information that controller changes  #控制器更改的状态信息

#endif
