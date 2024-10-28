#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include "query.h"

namespace py = pybind11;

#define MAX_BUFFER_SIZE 512 
#define SERIAL_INTERVAL_SEND 4000
#define SERIAL_INTERVAL_RESP 4000
#define WHEEL_D 0.311
#define WHEEL_T 0.461
#define GEAR_RATIO 50.0 
#define STEP_RESOLUTION 0.01/180*M_PI

struct ODOMETORY {
  double dist_R;
  double dist_L;
  double travel;
  double rotation;
  double rx;
  double ry;
  double ra;
  ODOMETORY() {
    dist_R = 0.0;
    dist_L = 0.0;
    travel = 0.0;
    rotation = 0.0;
    rx = 0.0;
    ry = 0.0;
    ra = 0.0;
  }
};

void read_res(int fd, uint8_t *buf, int length);
void show_state(int fd);
void read_odo(int fd, uint8_t *buf, ODOMETORY &odo);
void read_state(int fd);

// CRC create
void calcBcc(uint8_t *sendData, int length) {
  unsigned int crcH, crcL;
  int crc=0xFFFF;
  for (int no = 0; no < length-2; no++) {
    crc = crc ^ sendData[no];
    for (int i = 0; i < 8; i++) {
      if (1 == crc % 2) {
        crc = crc >> 1;
        crc = 0xA001 ^ crc;
      } else {
        crc = crc >> 1;
      }
    }
  }
  crcL = (crc & 255);
  crcH = (crc >> 8 & 255);
  sendData[length-2] = static_cast<unsigned char>(crcL);
  sendData[length-1] = static_cast<unsigned char>(crcH);
}

void send_cmd(int fd, uint8_t *cmd, int length) {
  calcBcc(cmd, length);
#ifdef DEBUG_SENDRESP
  std::cerr << "[SEND]";
  for (int i = 0; i < length; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd[i]) << " ";
  }
  std::cerr << "\n";
#endif
  int n = write(fd, cmd, length);
  usleep(SERIAL_INTERVAL_SEND);
}

void simple_send_cmd(int fd, uint8_t *cmd, int length) {
  send_cmd(fd, cmd, length);
  uint8_t res_buf[MAX_BUFFER_SIZE];
  read_res(fd, res_buf, 8);
}

int ReadByte(int fd, uint8_t *buf) {
  return read(fd, buf, sizeof(uint8_t));
}

void read_res(int fd, uint8_t *buf2, int length) {
  int tries = 3;
  int tmp_len = 0;
  uint8_t buf[1];
  while (tries) {
    if(ReadByte(fd, buf)) {
      buf2[tmp_len++] = buf[0];
      if (tmp_len >= length) {
        break;
      }
    } else {
      tries++;
    }
  }
  usleep(SERIAL_INTERVAL_RESP);
}

void turn_on_motors(int fd) {
  //std::cerr << "\033[12;1H" << "Turn ON RL...";
  simple_send_cmd(fd, Query_Write_Son_R, sizeof(Query_Write_Son_R));
  simple_send_cmd(fd, Query_Write_Son_L, sizeof(Query_Write_Son_L));
  //std::cerr << "Done.\n";
}

void turn_off_motors(int fd) {
  //std::cerr << "\033[12;1H" << "Turn OFF RL...";
  simple_send_cmd(fd, Query_Write_Soff_R, sizeof(Query_Write_Soff_R));
  simple_send_cmd(fd, Query_Write_Soff_L, sizeof(Query_Write_Soff_L));
  //std::cerr << "Done.\n";
}

void free_motors(int fd) {
  //std::cerr << "\033[12;1H" << "FREE RL...";
  simple_send_cmd(fd, Query_Write_FREE_R, sizeof(Query_Write_FREE_R));
  simple_send_cmd(fd, Query_Write_FREE_L, sizeof(Query_Write_FREE_L));
  //std::cerr << "Done.\n";
}

int init(std::string SERIAL_PORT, int BAUDRATE) {
  std::cout << SERIAL_PORT << " " << BAUDRATE << "\n";
  int fd;
  if((fd = open(SERIAL_PORT.c_str(), O_RDWR | O_NOCTTY)) == -1) {
    std::cerr << "Can't open serial port\n";
    return false;
  } else {
    std::cerr << "Get fd: " << fd << "\n";
  }
  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD | PARENB;
  tio.c_iflag &= ~ICRNL;
  tio.c_iflag &= ~INLCR;
  tio.c_cc[VTIME] = 1;
  cfsetispeed(&tio, BAUDRATE);
  cfsetospeed(&tio, BAUDRATE);
  tcsetattr(fd, TCSANOW, &tio);

  std::cerr << "ID Share configration...";
  simple_send_cmd(fd, Query_IDshare_R, sizeof(Query_IDshare_R));
  simple_send_cmd(fd, Query_IDshare_L, sizeof(Query_IDshare_L));
  simple_send_cmd(fd, Query_READ_R,    sizeof(Query_READ_R));
  simple_send_cmd(fd, Query_READ_L,    sizeof(Query_READ_L));
  simple_send_cmd(fd, Query_WRITE_R,   sizeof(Query_WRITE_R));
  simple_send_cmd(fd, Query_WRITE_L,   sizeof(Query_WRITE_L));
  std::cerr << "Done.\n";

  return fd;
  //return std::make_pair(2, 2.0);
}

void read_state(int fd) {
  uint8_t buf[MAX_BUFFER_SIZE];
  //send_cmd(Query_NET_ID_READ_ODO, sizeof(Query_NET_ID_READ_ODO));
  //read_res(buf, 17);
  send_cmd(fd, Query_NET_ID_READ, sizeof(Query_NET_ID_READ));
  read_res(fd, buf, 57);
#if 0
  std::cerr << "\033[11A";
  std::cerr << "Read state\n";
  show_state(buf);
#endif
  //return buf;
  //read_odo(buf, odo);

  int OFFSET = 26;
  int alarm_code_R     = static_cast<int>(buf[ 3] << 24 | buf[ 4] << 16 | buf[ 5] << 8 | buf[ 6]);
  double temp_driver_R = static_cast<int>(buf[ 7] << 24 | buf[ 8] << 16 | buf[ 9] << 8 | buf[10]) * 0.1;
  double temp_motor_R  = static_cast<int>(buf[11] << 24 | buf[12] << 16 | buf[13] << 8 | buf[14]) * 0.1;
  int position_R       = static_cast<int>(buf[15] << 24 | buf[16] << 16 | buf[17] << 8 | buf[18]);
  int power_R          = static_cast<int>(buf[19] << 24 | buf[20] << 16 | buf[21] << 8 | buf[22]);
  double voltage_R     = static_cast<int>(buf[23] << 24 | buf[24] << 16 | buf[25] << 8 | buf[26]) * 0.1;

  int alarm_code_L     = static_cast<int>(buf[ 3 + OFFSET] << 24 | buf[ 4 + OFFSET] << 16 | buf[ 5 + OFFSET] << 8 | buf[ 6 + OFFSET]);
  double temp_driver_L = static_cast<int>(buf[ 7 + OFFSET] << 24 | buf[ 8 + OFFSET] << 16 | buf[ 9 + OFFSET] << 8 | buf[10 + OFFSET]) * 0.1;
  double temp_motor_L  = static_cast<int>(buf[11 + OFFSET] << 24 | buf[12 + OFFSET] << 16 | buf[13 + OFFSET] << 8 | buf[14 + OFFSET]) * 0.1;
  int position_L       = static_cast<int>(buf[15 + OFFSET] << 24 | buf[16 + OFFSET] << 16 | buf[17 + OFFSET] << 8 | buf[18 + OFFSET]);
  int power_L          = static_cast<int>(buf[19 + OFFSET] << 24 | buf[20 + OFFSET] << 16 | buf[21 + OFFSET] << 8 | buf[22 + OFFSET]);
  double voltage_L     = static_cast<int>(buf[23 + OFFSET] << 24 | buf[24 + OFFSET] << 16 | buf[25 + OFFSET] << 8 | buf[26 + OFFSET]) * 0.1;

  double dist_L = position_L * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double dist_R = position_R * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double travel = (dist_L + dist_R)/2.0;
  double rotation = (dist_R - dist_L)/WHEEL_T;
  double voltage = (voltage_L + voltage_R)/2.0;

  //show_state(buf);
}

void show_state(int fd) {
  uint8_t buf[MAX_BUFFER_SIZE];
  send_cmd(fd, Query_NET_ID_READ, sizeof(Query_NET_ID_READ));
  read_res(fd, buf, 57);

  int OFFSET = 26;
  int alarm_code_R     = static_cast<int>(buf[ 3] << 24 | buf[ 4] << 16 | buf[ 5] << 8 | buf[ 6]);
  double temp_driver_R = static_cast<int>(buf[ 7] << 24 | buf[ 8] << 16 | buf[ 9] << 8 | buf[10]) * 0.1;
  double temp_motor_R  = static_cast<int>(buf[11] << 24 | buf[12] << 16 | buf[13] << 8 | buf[14]) * 0.1;
  int position_R       = static_cast<int>(buf[15] << 24 | buf[16] << 16 | buf[17] << 8 | buf[18]);
  int power_R          = static_cast<int>(buf[19] << 24 | buf[20] << 16 | buf[21] << 8 | buf[22]);
  double voltage_R     = static_cast<int>(buf[23] << 24 | buf[24] << 16 | buf[25] << 8 | buf[26]) * 0.1;

  int alarm_code_L     = static_cast<int>(buf[ 3 + OFFSET] << 24 | buf[ 4 + OFFSET] << 16 | buf[ 5 + OFFSET] << 8 | buf[ 6 + OFFSET]);
  double temp_driver_L = static_cast<int>(buf[ 7 + OFFSET] << 24 | buf[ 8 + OFFSET] << 16 | buf[ 9 + OFFSET] << 8 | buf[10 + OFFSET]) * 0.1;
  double temp_motor_L  = static_cast<int>(buf[11 + OFFSET] << 24 | buf[12 + OFFSET] << 16 | buf[13 + OFFSET] << 8 | buf[14 + OFFSET]) * 0.1;
  int position_L       = static_cast<int>(buf[15 + OFFSET] << 24 | buf[16 + OFFSET] << 16 | buf[17 + OFFSET] << 8 | buf[18 + OFFSET]);
  int power_L          = static_cast<int>(buf[19 + OFFSET] << 24 | buf[20 + OFFSET] << 16 | buf[21 + OFFSET] << 8 | buf[22 + OFFSET]);
  double voltage_L     = static_cast<int>(buf[23 + OFFSET] << 24 | buf[24 + OFFSET] << 16 | buf[25 + OFFSET] << 8 | buf[26 + OFFSET]) * 0.1;

  double dist_L = position_L * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double dist_R =-position_R * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  double travel = (dist_L + dist_R)/2.0;
  double rotation = (dist_R - dist_L)/WHEEL_T;

  std::cerr << "\033[1;1H" << "-------------";
  std::cerr << "\033[2;1H" << "Alarm_L:" << alarm_code_L;
  std::cerr << "\033[3;1H" << "Driver_L temp:" << std::dec << temp_driver_L;
  std::cerr << "\033[4;1H" << "Motor_L  temp:" << std::dec << temp_motor_L;
  std::cerr << "\033[5;1H" << "Position_L:" << position_L * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  std::cerr << "\033[6;1H" << "Power_L:" << power_L;
  std::cerr << "\033[7;1H" << "Voltage_L:" << voltage_R;

  std::cerr << "\033[2;40H" <<  "Alarm_R:" << alarm_code_R;
  std::cerr << "\033[3;40H" <<  "Driver_R temp:" << std::dec << temp_driver_R;
  std::cerr << "\033[4;40H" <<  "Motor_R  temp:" << std::dec << temp_motor_R;
  std::cerr << "\033[5;40H" <<  "Position_R:" << position_R * STEP_RESOLUTION * 0.5*WHEEL_D / GEAR_RATIO;
  std::cerr << "\033[6;40H" <<  "Power_R:" << power_R;
  std::cerr << "\033[7;40H" <<  "Voltage_R:" << voltage_R;
  std::cerr << "\033[8;1H\033[2K" << travel << " " << rotation * 180.0/M_PI;
    //<< " " << shm_loc->x << " " << shm_loc->y << " " << shm_loc->a * 180/M_PI;
  std::cerr << "\033[9;1H" << "-------------\n";
}

void calc_vw2hex(uint8_t *Query_NET_ID_WRITE, double v, double w) {
  double wr = v/(WHEEL_D/2) + w*WHEEL_T/(1.0*WHEEL_D);
  double wl = v/(WHEEL_D/2) - w*WHEEL_T/(1.0*WHEEL_D);
  double motor_wr_rpm =-wr / 2 / M_PI * static_cast<double>(GEAR_RATIO) * 60;
  double motor_wl_rpm = wl / 2 / M_PI * static_cast<double>(GEAR_RATIO) * 60;
  Query_NET_ID_WRITE[15] = (static_cast<int>(motor_wr_rpm) >> 24) & 0xFF;
  Query_NET_ID_WRITE[16] = (static_cast<int>(motor_wr_rpm) >> 16) & 0xFF;
  Query_NET_ID_WRITE[17] = (static_cast<int>(motor_wr_rpm) >>  8) & 0xFF;
  Query_NET_ID_WRITE[18] =  static_cast<int>(motor_wr_rpm)        & 0xFF;
  Query_NET_ID_WRITE[39] = (static_cast<int>(motor_wl_rpm) >> 24) & 0xFF;
  Query_NET_ID_WRITE[40] = (static_cast<int>(motor_wl_rpm) >> 16) & 0xFF;
  Query_NET_ID_WRITE[41] = (static_cast<int>(motor_wl_rpm) >>  8) & 0xFF;
  Query_NET_ID_WRITE[42] =  static_cast<int>(motor_wl_rpm)        & 0xFF;
#if 0
  for (int i = 15; i < 19; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(Query_NET_ID_WRITE[i]) << " ";
  }
  std::cerr << "\n";
  for (int i = 39; i < 43; i++) {
    std::cerr << std::hex << std::setw(2) << std::setfill('0')
      << static_cast<int>(Query_NET_ID_WRITE[i]) << " ";
  }
  std::cerr << "\n";
#endif
}

void send_vw(int fd, double v, double w) {
  calc_vw2hex(Query_NET_ID_WRITE, v, w);
  simple_send_cmd(fd, Query_NET_ID_WRITE, sizeof(Query_NET_ID_WRITE));
}

// Pythonモジュールを定義
PYBIND11_MODULE(MotorDriver, m) {
    m.def("init", &init, "initialize motor driver in C++");
    m.def("turn_on_motors", &turn_on_motors, "turn on the motor in C++");
    m.def("turn_off_motors", &turn_off_motors, "turn off the motor in C++");
    m.def("free_motors", &free_motors, "free the motor in C++");
    m.def("read_state", &read_state, "read status of the motor in C++");
    m.def("show_state", &show_state, "show status of the motor in C++");
    m.def("send_vw", &send_vw, "send v and w command to motors in C++");
}
