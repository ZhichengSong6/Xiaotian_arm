#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include "arm_controller.h"

// Control table address
#define ADDR_TORQUE_ENABLE          64
#define ADDR_GOAL_POSITION          116
#define ADDR_PRESENT_POSITION       132
#define ADDR_PRESENT_CURRENT        126
#define ADDR_GOAL_CURRENT           102
#define ADDR_OPERATE_MODE           11

// Data Byte Length
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4
#define LEN_PRESENT_CURRENT             2

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
static int DXL_ID = 1;
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/Dynamixel"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define OPERATE_MODE                    0
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold



int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

int main()
{
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandinclude_directories("./include/dynamixel_sdk")
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  
  dynamixel::ArmController armController(portHandler, packetHandler);

  armController.portSetUp(BAUDRATE);
  // armController.operateModeChanging(DXL_ID, ADDR_OPERATE_MODE, OPERATE_MODE);
  armController.torqueEnable(DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

  // armController.initialPosition(DXL_ID, ADDR_GOAL_POSITION ,ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT);
  armController.grasping(DXL_ID, ADDR_GOAL_POSITION ,ADDR_PRESENT_POSITION, ADDR_PRESENT_CURRENT);
  // armController.grasping1(DXL_ID, ADDR_GOAL_CURRENT, ADDR_PRESENT_CURRENT);

  // Disable Dynamixel Torque
  armController.torqueDisable(DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  // Close port
  armController.closePort();

  return 0;
}
