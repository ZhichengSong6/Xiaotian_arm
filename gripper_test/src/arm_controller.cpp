#include "arm_controller.h"

using namespace dynamixel;

ArmController::ArmController(PortHandler *port, PacketHandler *ph)
  : port_(port),
    ph_(ph),
    last_result_(false),
    is_param_changed_(false),
    param_(0)
{
  clearParam();
}

void ArmController::portSetUp(int baudrate){
    if (port_->openPort())
    {
    printf("Succeeded to open the port!\n");
    }
    else
    {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return;
    }

    // Set port baudrate
    if (port_->setBaudRate(baudrate))
    {
    printf("Succeeded to change the baudrate!\n");
    }
    else
    {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return;
    }
}

void ArmController::torqueEnable(int id, uint16_t address, uint8_t data){
    dxl_comm_result = ph_->write1ByteTxRx(port_, id, address, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
    printf("1%s\n", ph_->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
    printf("2%s\n", ph_->getRxPacketError(dxl_error));
    }
    else
    {
    printf("Dynamixel#%d has been successfully connected \n", id);
    }
}

void ArmController::torqueDisable(int id, uint16_t address, uint8_t data){
    dxl_comm_result = ph_->write1ByteTxRx(port_, id, address, data, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
    printf("%s\n", ph_->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
    printf("%s\n", ph_->getRxPacketError(dxl_error));
    }
}

void ArmController::operateModeChanging(int id, uint16_t address, uint8_t data){
    dxl_comm_result = ph_->read1ByteTxRx(port_, id, address, &operate_mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", ph_->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", ph_->getRxPacketError(dxl_error));
      }
    std::cout << "Present Control Mode: " << &operate_mode << std::endl;
}

void ArmController::writePosition(int id, uint16_t goal_address){
    dxl_comm_result = ph_->write4ByteTxRx(port_, id, goal_address, dxl_goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", ph_->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", ph_->getRxPacketError(dxl_error));
    }
}

void ArmController::writeCurrent(int id, uint16_t goal_address){
    dxl_comm_result = ph_->write4ByteTxRx(port_, id, goal_address, dxl_goal_current, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("%s\n", ph_->getTxRxResult(dxl_comm_result));
    }
    else if (dxl_error != 0)
    {
      printf("%s\n", ph_->getRxPacketError(dxl_error));
    }
}

void ArmController::readAndGetPosition(int id, uint16_t present_address){
    dxl_comm_result = ph_->read4ByteTxRx(port_, id, present_address, (uint32_t*)&dxl_present_position, &dxl_error);
     if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", ph_->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", ph_->getRxPacketError(dxl_error));
      }

    //   printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", id, dxl_goal_position, dxl_present_position);
}

void ArmController::readPresentCurrent(int id, uint16_t address){
    dxl_comm_result = ph_->read2ByteTxRx(port_, id, address, &dxl_present_current, &dxl_error);
     if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", ph_->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", ph_->getRxPacketError(dxl_error));
      }

      printf("[ID:%03d] Present Current:%03d\n", id, dxl_present_current);
}

void ArmController::initialPosition(int id, uint16_t goal_address, uint16_t present_address, uint16_t current_address){
    int dxl_initial_status_threshold = 1;
    int status_threshold = 0;
    while(1){
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;
        writePosition(id, goal_address);
        do{
            readAndGetPosition(id, present_address);
            readPresentCurrent(id, current_address);
            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", id, dxl_goal_position, dxl_present_position);
            status_threshold = abs(dxl_goal_position - dxl_present_position);
        }while(status_threshold > dxl_initial_status_threshold);
    }
}

void ArmController::grasping(int id, uint16_t goal_address, uint16_t present_address, uint16_t current_address){
    int dxl_initial_status_threshold = 1;
    int status_threshold = 0;
    while(1){
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;
        writePosition(id, goal_address);
        do{
            readAndGetPosition(id, present_address);
            readPresentCurrent(id, current_address);
            printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", id, dxl_goal_position, dxl_present_position);
            status_threshold = abs(dxl_goal_position - dxl_present_position);
            if(dxl_present_current > grasping_threshold[grasping_mode] && abs(dxl_present_current) < 2048){
                dxl_goal_position = dxl_present_position;
                writePosition(id, goal_address);
                grasping_mode += 1;
            }
        }while(status_threshold > dxl_initial_status_threshold);
    }
}

void ArmController::grasping1(int id, uint16_t goal_address, uint16_t present_address){
    while(1){
        printf("Press any key to continue! (or press ESC to quit!)\n");
        if (getch() == ESC_ASCII_VALUE)
            break;
        writeCurrent(id, goal_address);
        do{
            readPresentCurrent(id, present_address);
            printf("[ID:%03d] GoalCurrent:%03d  PresCurrent:%03d\n", id, grasping_threshold[grasping_mode], dxl_present_current);
        }while(true);
    }
}

void ArmController::closePort(){
    port_->closePort();
}

int ArmController::angleConvert(double angle){
    return (angle / 360 * 4095);
}

void ArmController::clearParam(){
    if (id_list_.size() == 0)
    return;

    for (unsigned int i = 0; i < id_list_.size(); i++)
    {
        delete[] data_list_[id_list_[i]];
        delete[] error_list_[id_list_[i]];
    }

    id_list_.clear();
    address_list_.clear();
    length_list_.clear();
    data_list_.clear();
    error_list_.clear();
    if (param_ != 0)
        delete[] param_;
    param_ = 0;
}

int ArmController::getch(){
    #if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
    #elif defined(_WIN32) || defined(_WIN64)
    return _getch();
    #endif
}