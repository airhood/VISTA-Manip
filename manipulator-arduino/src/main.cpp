#include <Arduino.h>
#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif


// #define ENABLE_DYNAMIXEL


void parseSerialPacket();
void sendSerialPacket(uint8_t command, const uint8_t* data, uint8_t data_len);


const uint8_t NUM_SERVOS = 6;

const float DXL_PROTOCOL_VERSION = 2.0;
const uint8_t NUM_DXL = 6;
const uint8_t DXL_ID_LIST[NUM_DXL] = {1,2,3,4,5,6};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint16_t SR_START_ADDR = 132;
const uint16_t SR_ADDR_LEN = 4;
const uint16_t SW_START_ADDR = 116;
const uint16_t SW_ADDR_LEN = 4;

typedef struct sr_data {
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data {
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sr_data_t sr_data[NUM_DXL];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[NUM_DXL];

sw_data_t sw_data[NUM_DXL];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[NUM_DXL];

DynamixelShield dxl;

using namespace ControlTableItem;

int32_t servo_positions[NUM_SERVOS] = {1024, 1024, 1024, 1024, 1024, 1024}; // 0 ~ 4095

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  soft_serial.begin(115200);

#ifdef ENABLE_DYNAMIXEL
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < NUM_DXL; i++) {
    uint8_t id = DXL_ID_LIST[i];

    if (dxl.ping(id)) {
      soft_serial.print("Dynamixel ID ");
      soft_serial.print(id);
      soft_serial.println(" found!");

      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
    } else {
      soft_serial.print("Dynamixel ID ");
      soft_serial.print(id);
      soft_serial.println(" not responding!");
    }
  }
  dxl.torqueOn(DXL_BROADCAST_ID);

  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;

  for (int i = 0; i < NUM_DXL; i++) {
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  for (int i = 0; i < NUM_DXL; i++) {
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
#endif
}

void loop() {
  if (soft_serial.available()) {
    parseSerialPacket();
    // clear leftover buffer
    while (soft_serial.available()) soft_serial.read();
  }

#ifdef ENABLE_DYNAMIXEL
  static uint32_t tick_count = 0;
  uint8_t recv_cnt;

  for (int i = 0; i < NUM_DXL; i++) {
    sw_data[i].goal_position = servo_positions[i];
  }

  sw_infos.is_info_changed = true;

  soft_serial.print("\n>>>>>> Manipulator Servo Instruction : ");
  soft_serial.println(tick_count++);

  if (dxl.syncWrite(&sw_infos) == true) {
    soft_serial.println("[SyncWrite] Success");
    for (int i = 0; i < sw_infos.xel_count; i++) {
      soft_serial.print("  ID: ");
      soft_serial.println(sw_infos.p_xels[i].id);
      soft_serial.print("\t Goal Position: ");
      soft_serial.println(sw_data[i].goal_position);
    }
  } else {
    soft_serial.print("[SyncWrite] Fail, Lib error code: ");
    soft_serial.print(dxl.getLastLibErrCode());
  }
  soft_serial.println();

  delay(250);

  // recv_cnt = dxl.syncRead(&sr_infos);
  // if(recv_cnt > 0) {
  //   soft_serial.print("[SyncRead] Success, Received ID Count: ");
  //   soft_serial.println(recv_cnt);
  //   for (int i = 0; i < recv_cnt; i++){
  //     soft_serial.print("  ID: ");
  //     soft_serial.print(sr_infos.p_xels[i].id);
  //     soft_serial.print(", Error: ");
  //     soft_serial.println(sr_infos.p_xels[i].error);
  //     soft_serial.print("\t Present Position: ");
  //     soft_serial.println(sr_data[i].present_position);
  //   }
  // }else{
  //   soft_serial.print("[SyncRead] Fail, Lib error code: ");
  //   soft_serial.println(dxl.getLastLibErrCode());
  // }

  for (uint8_t i = 0; i < NUM_DXL; i++) {
    soft_serial.print("ID ");
    soft_serial.print(DXL_ID_LIST[i]);
    soft_serial.print(": ");
    soft_serial.println(dxl.getPresentPosition(DXL_ID_LIST[i]));
  }
  
  soft_serial.println("=======================================================");
#endif

  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  delay(1);
}

void parseSerialPacket() {
  static uint8_t state = 0;
  static uint8_t length = 0;
  static uint8_t command = 0;
  static uint8_t data[32];
  static uint8_t data_idx = 0;
  static uint8_t crc = 0;

  while (soft_serial.available()) {
    uint8_t b = soft_serial.read();
    switch (state) {
      case 0:
        if (b == 0xFF) state = 1;
        break;
      case 1:
        length = b;
        data_idx = 0;
        state = 2;
        break;
      case 2:
        command = b;
        crc = b;
        state = 3;
        break;
      case 3:
        data[data_idx] = b;
        data_idx++;
        crc ^= b;
        if (data_idx >= length - 1) state = 4;
        break;
      case 4:
        if (b == crc) {
          switch (command) {
            case 0x01:
              for (int i = 0; i < NUM_SERVOS; i++) {
                servo_positions[i] = data[2*i] | (data[2*i+1]<<8);
              }
              break;
            case 0xFF:
              {
                digitalWrite(LED_BUILTIN, HIGH);
                uint8_t OK = 0x01;
                sendSerialPacket(0xFF, &OK, 1);
              }
              break;
          }
        }
        state = 0;
        break;
    }
  }
}

void sendSerialPacket(uint8_t command, const uint8_t* data, uint8_t data_len) {
  uint8_t length = 1 + data_len;
  uint8_t crc = command;

  soft_serial.write(0xFF);
  soft_serial.write(length);
  soft_serial.write(command);

  for (uint8_t i = 0; i < data_len; i++) {
    soft_serial.write(data[i]);
    crc ^= data[i];
  }

  soft_serial.write(crc);
}