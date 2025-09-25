// Firmware Version //
const float firmwareVersion = 1.0;

// Device Information //

const uint16_t tag = 11111;
const uint8_t devType = 107;

// Pin Definitions //
#define GPS_TX PC1
#define GPS_RX PC0
#define GPS_EN PB5
#define SCL_PIN PB15
#define SDA_PIN PA15
#define MOSI_PIN PA10
#define MISO_PIN PB14
#define SCK_PIN PB13
#define FSS_PIN PB9
#define ACC_INT1_PIN PB4
#define ACC_INT2_PIN PB3
#define CS_INT_PIN PA0

#define MAX_ELECTRODES 4

// Structs //

struct longPing{
    uint16_t ta;    
    uint16_t cnt;
    float la;
    float ln;
    uint8_t devtyp;
    bool mortality;
  }__attribute__((__packed__));

struct data{
    uint32_t datetime;
    uint16_t locktime;
    float lat;
    float lng;
    float hdop;
    byte id;
}__attribute__((__packed__));

struct reqPing{
    uint16_t tag;
    byte request;
  }__attribute__((__packed__));

struct setttings{
    uint16_t tag;
    int gpsFrq;
    int gpsTout;
    int hdop;
    int radioFrq;
    int startHour;
    int endHour;
    bool scheduled;
}__attribute__((__packed__));