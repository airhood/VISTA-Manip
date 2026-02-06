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

const uint8_t NUM_SERVOS = 6;

const uint8_t DXL_BROADCAST_ID = 254;
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

  DEBUG_SERIAL.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < NUM_DXL; i++) {
    uint8_t id = DXL_ID_LIST[i];

    if (dxl.ping(id)) {
      Serial.print("Dynamixel ID ");
      Serial.print(id);
      Serial.println(" found!");

      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
    } else {
      Serial.print("Dynamixel ID ");
      Serial.print(id);
      Serial.println(" not responding!");
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
}

void loop() {
  if (Serial.available()) {
    parseSerialCommand();
  }

  static uint32_t tick_count = 0;
  uint8_t recv_cnt;

  for (int i = 0; i < NUM_DXL; i++) {
    sw_data[i].goal_position = servo_positions[i];
  }

  sw_infos.is_info_changed = true;

  DEBUG_SERIAL.print("\n>>>>>> Manipulator Servo Instruction : ");
  DEBUG_SERIAL.println(tick_count++);

  if (dxl.syncWrite(&sw_infos) == true) {
    DEBUG_SERIAL.println("[SyncWrite] Success");
    for (int i = 0; i < sw_infos.xel_count; i++) {
      DEBUG_SERIAL.print("  ID: ");
      DEBUG_SERIAL.println(sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print("\t Goal Position: ");
      DEBUG_SERIAL.println(sw_data[i].goal_position);
    }
  } else {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

  delay(250);

  // recv_cnt = dxl.syncRead(&sr_infos);
  // if(recv_cnt > 0) {
  //   DEBUG_SERIAL.print("[SyncRead] Success, Received ID Count: ");
  //   DEBUG_SERIAL.println(recv_cnt);
  //   for (int i = 0; i < recv_cnt; i++){
  //     DEBUG_SERIAL.print("  ID: ");
  //     DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
  //     DEBUG_SERIAL.print(", Error: ");
  //     DEBUG_SERIAL.println(sr_infos.p_xels[i].error);
  //     DEBUG_SERIAL.print("\t Present Position: ");
  //     DEBUG_SERIAL.println(sr_data[i].present_position);
  //   }
  // }else{
  //   DEBUG_SERIAL.print("[SyncRead] Fail, Lib error code: ");
  //   DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  // }

  for (uint8_t i = 0; i < NUM_DXL; i++) {
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(DXL_ID_LIST[i]);
    DEBUG_SERIAL.print(": ");
    DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID_LIST[i]));
  }

  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  delay(50);
}

void parseSerialCommand() {
  static uint8_t state = 0;
  static uint8_t length = 0;
  static uint8_t command = 0;
  static uint8_t data[32];
  static uint8_t data_idx = 0;
  static uint8_t crc = 0;

  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch (state) {
      case 0:
        if (b = 0xFF) state = 1;
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
          for (int i = 0; i < NUM_SERVOS; i++) {
            servo_positions[i] = data[2*i] | (data[2*i+1]<<8);
          }
        }
        state = 0;
        break;
    }
  }
}
