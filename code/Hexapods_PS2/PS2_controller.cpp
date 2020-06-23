//====================================================================
//Project YFRobot Hexapods
//Description: Hexapods, control file.
//Hardware setup: PS2 version
//
//Walk method 1:    #步行方式1：
//- Left Stick Walk/Strafe
//- Right Stick Rotate
//
//Walk method 2:    #步行方式2：
//- Left Stick Disable
//- Right Stick Walk/Rotate
//
//PS2 CONTROLS:
//[Common Controls]
//- Start Turn on/off the bot-----------------------------------------------------------#Start   打开/关闭 机器人

/*********************************[Walk Controls #步行控制]*******************************/
//- select Switch gaits-----------------------------------------------------------------#select  切换[行走步态]
//- Left Stick(Walk mode 1) Walk/Strafe, (Walk mode 2) Disable--------------------------#左摇杆   （步行模式1）步行/冲压，（步行模式2）禁用
//- Right Stick(Walk mode 1) Rotate, (Walk mode 2) Walk/Rotate--------------------------#右摇杆   （步行模式1）旋转，（步行模式2）步行/旋转
//- R1 Toggle Double gait travel speed--------------------------------------------------#R1      切换双步行走速度 (两个速度切换)
//- R2 Toggle Double gait travel length-------------------------------------------------#R2      切换双步行走长度 (两个长度切换)

//- R3 Switch between Walk method 1 && Walk method 2------------------------------------#R3      切换[步行方式1/2]

/******************************[Shift Controls #Shift 控制]******************************/
//- L1 Toggle Shift mode----------------------------------------------------------------#L1      切换[shift 模式]
//- Left Stick Shift body X/Z-----------------------------------------------------------#左摇杆   移动机身X/Z
//- Right Stick Shift body Y and rotate body Y------------------------------------------#右摇杆   移动机身Y和旋转机身Y

/******************************[Rotate Controls #Rotate 控制]****************************/
//- L2 Toggle Rotate mode---------------------------------------------------------------#L2      切换[rotate 模式]
//- Left Stick Rotate body X/Z----------------------------------------------------------#左摇杆   旋转身体X / Z
//- Right Stick Rotate body Y-----------------------------------------------------------#右摇杆   旋转身体Y

/******************************[Single leg Controls #单腿控制]****************************/
//- Circle Toggle Single leg mode-------------------------------------------------------#○/圆    切换[单腿模式]
//- select Switch legs------------------------------------------------------------------#select  切换腿
//- Left Stick Move Leg X/Z (relative)--------------------------------------------------#左      移动腿X/Z(相对)
//- Right Stick Move Leg Y (absolute)---------------------------------------------------#右      移动腿Y(绝对)
//- R2 Hold/release leg position--------------------------------------------------------#R2      保持/释放腿部位置

/****************************[GP Player Controls #动作组播放控制 暂不支持]******************/
//- X Toggle GP Player Mode-------------------------------------------------------------#X       切换[动作组播放模式]
//- select Switch Sequences-------------------------------------------------------------#select  切换序列
//- R2 Start Sequence-------------------------------------------------------------------#R2      启动序列

/************************************[other #其它控制]************************************/
//- Square Toggle Balance mode----------------------------------------------------------#□/方    切换[平衡模式]
//- Triangle Move body to 35 mm from the  ground (walk pos) and back to the ground------#△/三角   将身体移至离地面35毫米（步行pos）并返回地面
//- D-Pad up Body up 10 mm--------------------------------------------------------------#↑       上升机身10毫米
//- D-Pad down Body down 10 mm----------------------------------------------------------#↓       下降机身向下10毫米
//- D-Pad left tdecrease speed with 50mS------------------------------------------------#←       以50mS降低速度
//- D-Pad righ tincrease speed with 50mS------------------------------------------------#→       以50mS的速度增加速度
//
//====================================================================
// [Include files]
#if ARDUINO>99
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

#include "Hex_Globals.h"

#ifdef USEPS2
#include <PS2X_lib.h>

//[CONSTANTS]
#define WALKMODE          0
#define TRANSLATEMODE     1
#define ROTATEMODE        2
#define SINGLELEGMODE     3
#define GPPLAYERMODE      4

#define cTravelDeadZone 4      // The deadzone for the analog input from the remote                       #来自遥控器的模拟输入的死区
#define  MAXPS2ERRORCNT  5     // How many times through the loop will we go before shutting off robot?   #在关闭机器人之前，我们会通过循环多少次？

#ifndef MAX_BODY_Y
#define MAX_BODY_Y 100
#endif

//=============================================================================
// Global - Local to this file only...  #全局 - 仅限本地文件
//=============================================================================
PS2X ps2x; // create PS2 Controller Class #创建PS2控制器类

// Define an instance of the Input Controller... #定义输入控制器的实例...
InputController  g_InputController;       // Our Input controller #输入控制器

static short       g_BodyYOffset;
static short       g_sPS2ErrorCnt;
static short       g_BodyYShift;
static byte        ControlMode;
static bool        DoubleHeightOn;
static bool        DoubleTravelOn;
static bool        WalkMethod;
byte               GPSeq;                // Number of the sequence                                    #序列号
short              g_sGPSMController;    // What GPSM value have we calculated. 0xff - Not used yet   #我们计算了什么GPSM值。 0xff - 尚未使用

// some external or forward function references.
extern void PS2TurnRobotOff(void);

//==============================================================================
// This is The function that is called by the Main program to initialize
//the input controller, which in this case is the PS2 controller
//process any commands.
// #这是Main程序调用的初始化输入控制器的功能，在这种情况下，PS2控制器处理任何命令。
//==============================================================================

// If both PS2 and XBee are defined then we will become secondary to the xbee
//  #如果同时定义了PS2和XBee，那么我们将会继续使用xbee
void InputController::Init(void) {
  int error;

  //error = ps2x.config_gamepad(57, 55, 56, 54);  // Setup gamepad (clock, command, attention, data) pins
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT);  // Setup gamepad (clock, command, attention, data) pins

  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  g_sPS2ErrorCnt = 0;  // error count

  ControlMode = WALKMODE;
  DoubleHeightOn = false;
  DoubleTravelOn = false;
  WalkMethod = false;

  g_InControlState.SpeedControl = 100;    // Sort of migrate stuff in from Devon.
}

//==============================================================================
// This function is called by the main code to tell us when it is about to
// do a lot of bit-bang outputs and it would like us to minimize any interrupts
// that we do while it is active...
//  #这个函数被主代码调用告诉我们它什么时候要做很多bit-bang输出，并且它希望我们尽量减少我们在它处于活动状态时所做的任何中断。
//==============================================================================
void InputController::AllowControllerInterrupts(boolean fAllow) {
  // We don't need to do anything...
}

//==============================================================================
// This is The main code to input function to read inputs from the PS2 and then
//process any commands.
// #这是输入函数从PS2读取输入并处理任何命令的主要代码。
//==============================================================================
void InputController::ControlInput(void)
{
  boolean fAdjustLegPositions = false;
  // Then try to receive a packet of information from the PS2.
  // #然后尝试从PS2接收信息包
  ps2x.read_gamepad();          //read controller and set large motor to spin at 'vibrate' speed #读取控制器并设置大型电机以“振动”速度旋转

  // Wish the library had a valid way to verify that the read_gamepad succeeded... Will hack for now
  if ((ps2x.Analog(1) & 0xf0) == 0x70) {
    // In an analog mode so should be OK... #在模拟模式下，所以应该是OK ...
    g_sPS2ErrorCnt = 0;    // clear out error count... #清除错误计数...

    if (ps2x.ButtonPressed(PSB_START)) {    // OK lets try "start" button for Start.
      if (g_InControlState.fHexOn) {
        PS2TurnRobotOff();
      } else {
        //Turn on #打开
        g_InControlState.fHexOn = 1;
        fAdjustLegPositions = true;
      }
    }

    if (g_InControlState.fHexOn) {
      // [SWITCH MODES #模式开关]

      //Translate mode #翻转模式
      if (ps2x.ButtonPressed(PSB_L1)) {   // L1 Button Test  #L1按钮测试
        MSound( 1, 50, 2000);
        if (ControlMode != TRANSLATEMODE )
          ControlMode = TRANSLATEMODE;
        else {
          if (g_InControlState.SelectedLeg == 255)
            ControlMode = WALKMODE;
          else
            ControlMode = SINGLELEGMODE;
        }
      }

      //Rotate mode  #旋转模式
      if (ps2x.ButtonPressed(PSB_L2)) {   // L2 Button Test  #L2按钮测试
        MSound( 1, 50, 2000);
        if (ControlMode != ROTATEMODE)
          ControlMode = ROTATEMODE;
        else {
          if (g_InControlState.SelectedLeg == 255)
            ControlMode = WALKMODE;
          else
            ControlMode = SINGLELEGMODE;
        }
      }

      //Single leg mode fNO  #单腿模式
      if (ps2x.ButtonPressed(PSB_CIRCLE)) {   // ○ - Circle Button Test
        if (abs(g_InControlState.TravelLength.x) < cTravelDeadZone && abs(g_InControlState.TravelLength.z) < cTravelDeadZone
            && abs(g_InControlState.TravelLength.y * 2) < cTravelDeadZone )   {
          if (ControlMode != SINGLELEGMODE) {
            ControlMode = SINGLELEGMODE;
            if (g_InControlState.SelectedLeg == 255)  //Select leg if none is selected
              g_InControlState.SelectedLeg = cRF;     //Start leg
            MSound(1, 100, 4000);
          } else {
            ControlMode = WALKMODE;
            g_InControlState.SelectedLeg = 255;
            MSound(1, 50, 2000);
          }
        }
      }

#ifdef OPT_GPPLAYER
      // GP Player Mode X #动作组播放模式
      if (ps2x.ButtonPressed(PSB_CROSS)) {    // X - Cross Button Test
        if (ControlMode != GPPLAYERMODE) {
          ControlMode = GPPLAYERMODE;
          GPSeq = 0;
          MSound(1, 50, 2000);
        }
        else
          ControlMode = WALKMODE;
      }
#endif // OPT_GPPLAYER

      //[Common functions #常用功能]
      //Switch Balance mode on/off #平衡模式开关
      if (ps2x.ButtonPressed(PSB_SQUARE)) {   // □ - Square Button Test
        g_InControlState.BalanceMode = !g_InControlState.BalanceMode;
        if (g_InControlState.BalanceMode) {
          MSound(1, 250, 1500);
        } else {
          MSound( 2, 100, 2000, 50, 4000);
        }
      }

      //Stand up, sit down  -  站起来坐下
      if (ps2x.ButtonPressed(PSB_TRIANGLE)) { // △ - Triangle - Button Test
        if (g_BodyYOffset > 0)
          g_BodyYOffset = 0;
        else
          g_BodyYOffset = 35;
        fAdjustLegPositions = true;
      }

      if (ps2x.ButtonPressed(PSB_PAD_UP)) {   // ↑ - D-Up - Button Test
        g_BodyYOffset += 10;
        // And see if the legs should adjust...   - 看看腿是否应该调整...
        fAdjustLegPositions = true;
        if (g_BodyYOffset > MAX_BODY_Y)
          g_BodyYOffset = MAX_BODY_Y;
      }

      if (ps2x.ButtonPressed(PSB_PAD_DOWN) && g_BodyYOffset) {    // ↓ - D-Down - Button Test
        if (g_BodyYOffset > 10)
          g_BodyYOffset -= 10;
        else
          g_BodyYOffset = 0;      // constrain don't go less than zero.  - 约束不要小于零。

        // And see if the legs should adjust...
        fAdjustLegPositions = true;
      }

      // 增减速
      if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {    // → - D-Right - Button Test
        if (g_InControlState.SpeedControl > 0) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl - 50;
          MSound( 1, 50, 2000);
        }
      }

      if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {     // ← - D-Left - Button Test
        if (g_InControlState.SpeedControl < 2000 ) {
          g_InControlState.SpeedControl = g_InControlState.SpeedControl + 50;
          MSound( 1, 50, 2000);
        }
      }

      //[Walk functions - 行走函数]
      if (ControlMode == WALKMODE) {
        //Switch gates - 步态开关.
        if (ps2x.ButtonPressed(PSB_SELECT)            // Select Button Test
            && abs(g_InControlState.TravelLength.x) < cTravelDeadZone               //No movement
            && abs(g_InControlState.TravelLength.z) < cTravelDeadZone
            && abs(g_InControlState.TravelLength.y * 2) < cTravelDeadZone  ) {
          g_InControlState.GaitType = g_InControlState.GaitType + 1;                // Go to the next gait...
          if (g_InControlState.GaitType < NUM_GAITS) {                              // Make sure we did not exceed number of gaits...
            MSound( 1, 50, 2000);
          } else {
            MSound(2, 50, 2000, 50, 2250);
            g_InControlState.GaitType = 0;
          }
          GaitSelect();
        }

        //Double leg lift height - 双脚举起高度
        if (ps2x.ButtonPressed(PSB_R1)) {   // R1 Button Test
          MSound( 1, 50, 2000);
          DoubleHeightOn = !DoubleHeightOn;
          if (DoubleHeightOn)
            g_InControlState.LegLiftHeight = 80;
          else
            g_InControlState.LegLiftHeight = 50;
        }

        //Double Travel Length - 双行程长度
        if (ps2x.ButtonPressed(PSB_R2)) {   // R2 Button Test
          //          MSound(1, 50, 2000);
          DoubleTravelOn = !DoubleTravelOn;
          if (DoubleTravelOn)
            MSound(1, 50, 2000);
          else
            MSound(2, 50, 2000, 20, 1000);
        }

        // Switch between Walk method 1 && Walk method 2
        if (ps2x.ButtonPressed(PSB_R3)) {   // R3 Button Test
          //          MSound(1, 50, 2000);
          WalkMethod = !WalkMethod;
          if (DoubleTravelOn)
            MSound(1, 50, 2000);
          else
            MSound(2, 50, 2000, 20, 1000);
        }

        //Walking #行走
        if (WalkMethod)  //(Walk Methode) #行走方式
          g_InControlState.TravelLength.z = (ps2x.Analog(PSS_RY) - 128);      //Right Stick Up/Down
        else {
          g_InControlState.TravelLength.x = -(ps2x.Analog(PSS_LX) - 128);
          g_InControlState.TravelLength.z = (ps2x.Analog(PSS_LY) - 128);
        }

        if (!DoubleTravelOn) {  //(Double travel length)
          g_InControlState.TravelLength.x = g_InControlState.TravelLength.x / 2.5;
          g_InControlState.TravelLength.z = g_InControlState.TravelLength.z / 2.5;
        }
        else {
          g_InControlState.TravelLength.x = g_InControlState.TravelLength.x / 1.5;
          g_InControlState.TravelLength.z = g_InControlState.TravelLength.z / 1.5;
        }

        g_InControlState.TravelLength.y = -(ps2x.Analog(PSS_RX) - 128) / 4;     //Right Stick Left/Right
      }

      //[Translate functions #Translate 功能]
      g_BodyYShift = 0;
      if (ControlMode == TRANSLATEMODE) {
        g_InControlState.BodyPos.x = (ps2x.Analog(PSS_LX) - 128) / 2;
        g_InControlState.BodyPos.z = -(ps2x.Analog(PSS_LY) - 128) / 3;
        g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128) * 2;
        g_BodyYShift = (-(ps2x.Analog(PSS_RY) - 128) / 2);
      }

      //[Rotate functions #Rotate 功能]
      if (ControlMode == ROTATEMODE) {
        g_InControlState.BodyRot1.x = (ps2x.Analog(PSS_LY) - 128);
        g_InControlState.BodyRot1.y = (ps2x.Analog(PSS_RX) - 128) * 2;
        g_InControlState.BodyRot1.z = (ps2x.Analog(PSS_LX) - 128);
        g_BodyYShift = (-(ps2x.Analog(PSS_RY) - 128) / 2);
      }

      //[Single leg functions #单腿 功能]
      if (ControlMode == SINGLELEGMODE) {
        //Switch leg for single leg control
        if (ps2x.ButtonPressed(PSB_SELECT)) {       // Select Button Test
          MSound(1, 50, 2000);
          if (g_InControlState.SelectedLeg < 5)
            g_InControlState.SelectedLeg = g_InControlState.SelectedLeg + 1;
          else
            g_InControlState.SelectedLeg = 0;
        }

        g_InControlState.SLLeg.x = (ps2x.Analog(PSS_LX) - 128) / 2;     //Left Stick Right/Left
        g_InControlState.SLLeg.y = (ps2x.Analog(PSS_RY) - 128) / 10;    //Right Stick Up/Down
        g_InControlState.SLLeg.z = (ps2x.Analog(PSS_LY) - 128) / 2;     //Left Stick Up/Down

        // Hold single leg in place
        if (ps2x.ButtonPressed(PSB_R2)) { // R2 Button Test
          MSound(1, 50, 2000);
          g_InControlState.fSLHold = !g_InControlState.fSLHold;
        }
      }

#ifdef OPT_GPPLAYER
      //[GPPlayer functions #GP播放器 功能]
      if (ControlMode == GPPLAYERMODE) {

        // Lets try some speed control... Map all values if we have mapped some before
        // or start mapping if we exceed some minimum delta from center
        // Have to keep reminding myself that commander library already subtracted 128...
        if (g_ServoDriver.FIsGPSeqActive() ) {
          if ((g_sGPSMController != 32767) || (ps2x.Analog(PSS_RY) > (128 + 16)) || (ps2x.Analog(PSS_RY) < (128 - 16))) {
            // We are in speed modify mode...  -  我们正处于速度修改模式...
            short sNewGPSM = map(ps2x.Analog(PSS_RY), 0, 255, -200, 200);
            if (sNewGPSM != g_sGPSMController) {
              g_sGPSMController = sNewGPSM;
              g_ServoDriver.GPSetSpeedMultiplyer(g_sGPSMController);
            }
          }
        }

        //Switch between sequences #在序列之间切换 切换动作组
        if (ps2x.ButtonPressed(PSB_SELECT)) {       // Select Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
            if (GPSeq < 5) {  //Max sequence
              MSound(1, 50, 1500);
              GPSeq = GPSeq + 1;
            } else {
              MSound(2, 50, 2000, 50, 2250);
              GPSeq = 0;
            }
          }
        }

        //Start Sequence #启动序列
        if (ps2x.ButtonPressed(PSB_R2))   // R2 Button Test
          if (!g_ServoDriver.FIsGPSeqActive() ) {
            g_ServoDriver.GPStartSeq(GPSeq);
            g_sGPSMController = 32767;  // Say that we are not in Speed modify mode yet... valid ranges are 50-200 (both postive and negative...
          } else {
            g_ServoDriver.GPStartSeq(0xff);    // tell the GP system to abort if possible...
            MSound (2, 50, 2000, 50, 2000);
          }
      }
#endif // OPT_GPPLAYER

      //Calculate walking time delay - 计算行走时间延迟
      g_InControlState.InputTimeDelay = 128 - max(max(abs(ps2x.Analog(PSS_LX) - 128), abs(ps2x.Analog(PSS_LY) - 128)), abs(ps2x.Analog(PSS_RX) - 128));
    }

    //Calculate g_InControlState.BodyPos.y
    g_InControlState.BodyPos.y = min(max(g_BodyYOffset + g_BodyYShift,  0), MAX_BODY_Y);
    if (fAdjustLegPositions)
      AdjustLegPositionsToBodyHeight();    // Put main workings into main program file  #将主要工作放到主程序文件中
  } else {
    // We may have lost the PS2... See what we can do to recover...  #我们可能已经失去了PS2......看看我们能做些什么来恢复......
    if (g_sPS2ErrorCnt < MAXPS2ERRORCNT)
      g_sPS2ErrorCnt++;    // Increment the error count and if to many errors, turn off the robot.  #增加错误次数，如果遇到很多错误，请关闭机器人。
    else if (g_InControlState.fHexOn)
      PS2TurnRobotOff();
    ps2x.reconfig_gamepad();
  }
}

//==============================================================================
// PS2TurnRobotOff - code used couple of places so save a little room...
// PS2TurnRobotOff - 代码使用了几个地方，所以节省了少许空间....
//==============================================================================
void PS2TurnRobotOff(void)
{
  //Turn off
  g_InControlState.BodyPos.x = 0;
  g_InControlState.BodyPos.y = 0;
  g_InControlState.BodyPos.z = 0;
  g_InControlState.BodyRot1.x = 0;
  g_InControlState.BodyRot1.y = 0;
  g_InControlState.BodyRot1.z = 0;
  g_InControlState.TravelLength.x = 0;
  g_InControlState.TravelLength.z = 0;
  g_InControlState.TravelLength.y = 0;
  g_BodyYOffset = 0;
  g_BodyYShift = 0;
  g_InControlState.SelectedLeg = 255;
  g_InControlState.fHexOn = 0;
  AdjustLegPositionsToBodyHeight();    // Put main workings into main program file
}

#endif //
