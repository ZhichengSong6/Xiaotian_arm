#include "dynamixel_sdk.h"
#include <map>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#define STDIN_FILENO 0
#define ESC_ASCII_VALUE                 0x1b
#define MINIMUM_POSITION_LIMIT          0                   // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT        4095                // Refer to the Maximum Position Limit of product eManual

namespace  dynamixel
{
class ArmController
{
 private:
    PortHandler *port_;
    PacketHandler *ph_; 
    int initial_position;
    uint8_t param_goal_position;
    int dxl_comm_result = COMM_TX_FAIL;               // Communication result
    bool dxl_addparam_result = false;                 // addParam result
    bool dxl_getdata_result = false;                  // GetParam result
    std::vector<uint8_t>            id_list_;
    std::map<uint8_t, uint16_t>     address_list_;  // <id, start_address>
    std::map<uint8_t, uint16_t>     length_list_;   // <id, data_length>
    std::map<uint8_t, uint8_t *>    data_list_;     // <id, data>
    std::map<uint8_t, uint8_t *>    error_list_;    // <id, error>
    bool last_result_;
    bool is_param_changed_;
    uint8_t *param_;
    uint8_t dxl_error = 0;                            // Dynamixel error
 public:
    int32_t dxl_present_position = 0;
    uint16_t dxl_present_current = 0;
    uint32_t dxl_goal_current = 150;
    uint8_t operate_mode = -1;
    int grasping_mode = 3;
    int grasping_threshold[4] = {200, 250, 300, 350};
    int dxl_goal_position = 1700;   
    ArmController(PortHandler *port, PacketHandler *ph);
    ~ArmController(){ clearParam();}
    void portSetUp(int baudrate);
    void torqueEnable(int id, uint16_t address, uint8_t data);
    void torqueDisable(int id, uint16_t address, uint8_t data);
    void operateModeChanging(int id, uint16_t address, uint8_t data);
    void readAndGetPosition(int id, uint16_t present_address);
    void writePosition(int id, uint16_t goal_address);
    void writeCurrent(int id, uint16_t goal_address);
    void initialPosition(int id, uint16_t goal_address, uint16_t present_address, uint16_t current_address);
    void readPresentCurrent(int id, uint16_t address);
    void grasping(int id, uint16_t goal_address, uint16_t present_address, uint16_t current_address);
    void grasping1(int id, uint16_t goal_address, uint16_t present_address);
    int angleConvert(double angle);
    void closePort();
    void clearParam();
    int getch();
};
}
