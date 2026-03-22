#ifdef MAIN

#include <Arduino.h>
#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define SERIAL SerialUSB    
#else
  #define SERIAL Serial
#endif

#undef SERIAL
#define SERIAL Serial


// #define ENABLE_DYNAMIXEL

enum StatusLED { RED, YELLOW, GREEN, BLUE };
enum LEDState { OFF, ON, BLINK };

void parseSerialPacket();
void handleCommand(uint8_t command, uint8_t* data, uint8_t data_len);
void sendSerialPacket(uint8_t command, const uint8_t* data, uint8_t data_len);
void setStatusLED(StatusLED led, LEDState state, int blink_interval_ms = 500);
void tickStatusLED();
void tickHealthCheck();
void onSystemStatusUpdate();
void debugPrint(const char* fmt, ...);


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


const int RED_PIN    = 10;
const int YELLOW_PIN = 11;
const int GREEN_PIN  = 12;
const int BLUE_PIN   = 13;

const int LED_PINS[4] = { RED_PIN, YELLOW_PIN, GREEN_PIN, BLUE_PIN };

LEDState red_state;
LEDState yellow_state;
LEDState green_state;
LEDState blue_state;

LEDState led_state[4] = { OFF, OFF, OFF, OFF };
unsigned long led_blink_interval[4] = { 0, 0, 0, 0 };
unsigned long led_last_toggle[4] = { 0, 0, 0, 0 };
bool led_blink_on[4] = { false, false, false, false };


unsigned long health_check_request = 0;
const int HEALTH_CHECK_TIMEOUT_MS = 20;


enum RPiUartStatus { UART_DISCONNECTED, UART_CONNECTED };

typedef struct SystemStatus {
  RPiUartStatus rpi_uart;
} __attribute__((packed)) SystemStatus_t;

SystemStatus_t system_status;


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  SERIAL.begin(9600);

  pinMode(RED_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  
  digitalWrite(RED_PIN, LOW);
  digitalWrite(YELLOW_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);

#ifdef ENABLE_DYNAMIXEL
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < NUM_DXL; i++) {
    uint8_t id = DXL_ID_LIST[i];

    if (dxl.ping(id)) {
      debugPrint("Dynamixel ID ");
      debugPrint("%d", id);
      debugPrint(" found!\n");

      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_POSITION);
    } else {
      debugPrint("Dynamixel ID ");
      debugPrint("%d", id);
      debugPrint(" not responding!\n");
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

uint16_t dxl_tick = 0;

void loop() {
  Serial.println("hello world\n");
  SERIAL.println("hello world\n");
  debugPrint("hello world!\n");

  if (SERIAL.available()) {
    Serial.println("try parse");
    parseSerialPacket();
  }

  tickStatusLED();

#ifdef ENABLE_DYNAMIXEL
  if (dxl_tick == 250) {
    dxl_tick = 0;

    static uint32_t tick_count = 0;
    uint8_t recv_cnt;
  
    for (int i = 0; i < NUM_DXL; i++) {
      sw_data[i].goal_position = servo_positions[i];
    }
  
    sw_infos.is_info_changed = true;
  
    debugPrint("\n>>>>>> Manipulator Servo Instruction : ");
    debugPrint("%ld\n", tick_count++);
  
    if (dxl.syncWrite(&sw_infos) == true) {
      debugPrint("[SyncWrite] Success\n");
      for (int i = 0; i < sw_infos.xel_count; i++) {
        debugPrint("  ID: ");
        debugPrint("%d\n", sw_infos.p_xels[i].id);
        debugPrint("\t Goal Position: ");
        debugPrint("%ld\n", sw_data[i].goal_position);
      }
    } else {
      debugPrint("[SyncWrite] Fail, Lib error code: ");
      debugPrint("%ld", dxl.getLastLibErrCode());
    }
    debugPrint("\n");
  
    // recv_cnt = dxl.syncRead(&sr_infos);
    // if(recv_cnt > 0) {
    //   SERIAL.print("[SyncRead] Success, Received ID Count: ");
    //   SERIAL.println(recv_cnt);
    //   for (int i = 0; i < recv_cnt; i++){
    //     SERIAL.print("  ID: ");
    //     SERIAL.print(sr_infos.p_xels[i].id);
    //     SERIAL.print(", Error: ");
    //     SERIAL.println(sr_infos.p_xels[i].error);
    //     SERIAL.print("\t Present Position: ");
    //     SERIAL.println(sr_data[i].present_position);
    //   }
    // }else{
    //   SERIAL.print("[SyncRead] Fail, Lib error code: ");
    //   SERIAL.println(dxl.getLastLibErrCode());
    // }
  
    for (uint8_t i = 0; i < NUM_DXL; i++) {
      debugPrint("ID ");
      debugPrint("%d", DXL_ID_LIST[i]);
      debugPrint(": ");
      debugPrint("%d\n", dxl.getPresentPosition(DXL_ID_LIST[i]));
    }
    
    debugPrint("=======================================================\n");
  }
#endif

  // digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  delay(1);
  
  dxl_tick++;
}

void parseSerialPacket() {
  static uint8_t buf[64];
  static uint8_t buf_len = 0;

  while (SERIAL.available()) {
    if (buf_len < sizeof(buf)) {
      buf[buf_len++] = SERIAL.read();
    } else {
      SERIAL.read(); // 버퍼 꽉 차면 버림
    }
  }

  while (buf_len >= 4) {
    if (buf[0] != 0xFF) {
      memmove(buf, buf + 1, --buf_len);
      continue;
    }

    uint8_t length = buf[1];
    if (length == 0 || length > 32) {
      memmove(buf, buf + 1, --buf_len);
      continue;
    }

    uint8_t total_size = 1 + 1 + length + 1;
    if (buf_len < total_size) break;

    uint8_t command = buf[2];
    uint8_t* data = buf + 3;
    uint8_t crc = buf[total_size - 1];

    uint8_t calculated_crc = command;
    for (int i = 0; i < length - 1; i++) calculated_crc ^= data[i];

    if (crc == calculated_crc) {
      handleCommand(command, data, length - 1);
    }

    memmove(buf, buf + total_size, buf_len - total_size);
    buf_len -= total_size;
  }
}

void handleCommand(uint8_t command, uint8_t* data, uint8_t data_len) {
  switch (command) {
    case 0x01: // set servo positions
      for (int i = 0; i < NUM_SERVOS; i++) {
        servo_positions[i] = data[2*i] | (data[2*i+1]<<8);
      }
      break;
    case 0x03: // system status update request
      {
        uint8_t status = data[0];
      }
      break;
    case 0xFE: // health check reply
      {
        health_check_request = 0;
        RPiUartStatus prev = system_status.rpi_uart;
        system_status.rpi_uart = UART_CONNECTED;
        if (prev != system_status.rpi_uart) {
          onSystemStatusUpdate();
        }
      }
      break;
    case 0xFF: // health check request
      {
        digitalWrite(LED_BUILTIN, HIGH);
        health_check_request = millis();
        uint8_t OK = 0x01;
        sendSerialPacket(0xFF, &OK, 1);
      }
      break;
  }
}

void sendSerialPacket(uint8_t command, const uint8_t* data, uint8_t data_len) {
  uint8_t length = 1 + data_len;
  uint8_t crc = command;

  SERIAL.write(0xFF);
  SERIAL.write(length);
  SERIAL.write(command);

  for (uint8_t i = 0; i < data_len; i++) {
    SERIAL.write(data[i]);
    crc ^= data[i];
  }

  SERIAL.write(crc);
}

void setStatusLED(StatusLED led, LEDState state, int blink_interval_ms) {
  led_state[led] = state;
  if (state == BLINK) {
    led_blink_interval[led] = blink_interval_ms;
  }
}

void tickStatusLED() {
  unsigned long now = millis();
  for (int i = 0; i < 4; i++) {
    if (led_state[i] == ON) {
      digitalWrite(LED_PINS[i], HIGH);
    } else if (led_state[i] == OFF) {
      digitalWrite(LED_PINS[i], LOW);
    } else if (led_state[i] == BLINK) {
      if (now - led_last_toggle[i] >= led_blink_interval[i]) {
        led_blink_on[i] = !led_blink_on[i];
        digitalWrite(LED_PINS[i], led_blink_on[i] ? HIGH : LOW);
        led_last_toggle[i] = now;
      }
    }
  }
}

void tickHealthCheck() {
  unsigned long now = millis();
  if (now - health_check_request > HEALTH_CHECK_TIMEOUT_MS) {
    RPiUartStatus prev = system_status.rpi_uart;
    system_status.rpi_uart = UART_DISCONNECTED;
    if (prev != system_status.rpi_uart) {
      onSystemStatusUpdate();
    }
  }
}

void onSystemStatusUpdate() {
  sendSerialPacket(0x02, (uint8_t*)&system_status, sizeof(SystemStatus_t));
}

void debugPrint(const char* fmt, ...) {
  char buf[64];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  sendSerialPacket(0x10, (const uint8_t*)buf, strlen(buf));
}

#endif