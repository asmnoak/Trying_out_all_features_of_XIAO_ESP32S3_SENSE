//**********************************************************************************************************
//*    clock_wifi_radio with weekly schedule and experimental recording function 
//*                                       --  RDA5807 FM Radio which is controlled 
//*                                           by weekly schedule using XIAO ESP32S3. 
//*                                           Clock time of XIAO ESP32S3 refers NTP using wifi network. 
//*                                           This has recording WAV file and JPG file function 
//*                                           to SD micro card of XIAO ESP32S3 SENSE.
//*                                           Also support camera and mic functions.
//*
//**********************************************************************************************************
//  
//
//  2023/2/1 created by asmaro
//  2023/6/10 add server function
//  2023/6/23 add permanent data function
//  2025/10/20 change to apply recording WAV file function for XIAO ESP32S3 SENSE
//  2025/12/10 change to apply recording JPG file function for XIAO ESP32S3 SENSE
//  2026/1/10  change to support all functions of XIAO ESP32S3 SENSE,  such as camera and mic
//
#include "Arduino.h"
#include "Audio.h"
#include <TJpg_Decoder.h>
#include "esp_camera.h"       // esp lib
#include "SPI.h"
#include "FS.h"
#include "SD.h"
//#include "WiFi.h"
#include <WiFiMulti.h>
#include <WebServer.h>
#include "Wire.h"
#include <Adafruit_GFX.h>       // install using lib tool
//#include <Adafruit_SSD1306.h>   // install using lib tool
#include <Adafruit_SH110X.h>
//#include <Adafruit_ST7735.h>    // Hardware-specific library for ST7735
#include <esp_sntp.h>           // esp lib
#include <TimeLib.h>            // https://github.com/PaulStoffregen/Time
#include <RDA5807.h>            // install using lib tool
#include <Preferences.h>        //For permanent data
#include <driver/i2s.h>
#include <Adafruit_PCF8574.h>   // I/O expander
//#include <I2S.h>              // This has a poor function, so not used 

#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"
#define TFT_MOSI 9   //
#define TFT_SCLK 7   //
#define TFT_CS   3   // Chip select control pin 
#define TFT_DC   4   // Data Command control pin
#define TFT_RST  2

#define SHOOT_PIN  1   // To take a photo
#define LED_BUILTIN 21 // same as SD control pin


#define VERSION_NO   0.76
#define I2C_SDA      5          // I2C DATA
#define I2C_SCK      6          // I2C CLOCK
#define PIN_SDA  5              // xiao esp32s3 default
#define PIN_SCL  6              // xiao esp32s3 default
#define OLED_I2C_ADDRESS 0x3C   // Check the I2C bus of your OLED device
#define SCREEN_WIDTH 128        // OLED display width, in pixels
#define SCREEN_HEIGHT 64        // OLED display height, in pixels
#define OLED_RESET  -1          // Reset pin # (or -1 if sharing Arduino reset pin)
#define MAXSTNIDX    7          // station index 0-7          
#define MAXSCEDIDX   8          // schedule table index 0-8

#define RECORD_TIME   1         // in seconds, to estimate buffer full time
#define K32  32*1024            // 32KB

//#define SAMPLE_RATE 8000U
//#define SAMPLE_RATE 16000U
#define SAMPLE_RATE 32000U      // most applicable value
//#define SAMPLE_RATE 44100U
//#define SAMPLE_RATE 48000U
#define SAMPLE_BITS 16
#define WAV_HEADER_SIZE 44
#define CHAN_NUM    2           // channel number, stereo is 2 
#define SAMPLE_RATE_MIC 32000U  
// I2S to DAC ex. PCM5102
#define I2S_DOUT_A    1
#define I2S_BCLK_A    2
#define I2S_LRC_A     44
#define I2S_NUM_A     1        // DAC I2S port number -> not used currently
#define I2S_DOUT      2        // V 0.76 
#define I2S_BCLK      1        // v 0.76
#define I2S_LRC       44
// I2S from DSP Radio
#define I2S_DIN_S       4
#define I2S_BCLK_S      3                                  
#define I2S_LRC_S       43 
// SPI with SD card drive of esp32s3 sense
#define SD_CS         21
#define SPI_MOSI      9
#define SPI_MISO      8
#define SPI_SCK       7

// Wav File recording and reading
#define MAX_RECORD_TIME  30               // Max limited recording time in minutes
#define REC_FREQUENCY  20000000           // 1MHz-24MHz, apply for SPI & SD both
#define I2S_DMA_BUFFER  40                // number of  I2S_DMA_BUFFER ok:52
////
esp_err_t camera_enable_out_clock(camera_config_t *config);
void camera_disable_out_clock();
////
int photo_Count = 1;             // File Counter
bool camera_ok = false;          // Check camera status
bool sd_ok = false;              // Check sd status
bool shoot_s = false;            // shoot on/off by browser
//byte readArray[40000];           // 40k  for SVGA

bool stop_read = true;   // priority SD read active in loop()
bool REC_on = false;     // DSP recording on
bool REC_on_no_poff = false;    // recording on but not power off
bool MIC_rec_on = false; // MIC recording on
bool SD_open = false;    // SD open
bool SD_write = false;   // SD write ok
bool WAVE_HDR_write = false; // wavwfile headder wrote
bool I2S_err = false;    // any I2S(DSP) error detected
int last_blk = 0;
int avail_cnt = 0;
uint32_t record_size = (SAMPLE_RATE * SAMPLE_BITS * CHAN_NUM / 8) * RECORD_TIME; // possible size at once in byte
uint8_t *rec_buffer1 = NULL;  // PSRAM
uint8_t *rec_buffer2 = NULL;  // PSRAM
uint8_t *rec_buffer32k = NULL;  // PSRAM
uint8_t *jpg_buffer = NULL;   // PSRAM
int curr_buf = 1;             // current buff
uint32_t recorded_size = 0;
uint32_t total_recorded_size = 0;
uint32_t estimated_recorded_size = 0;
File file;
char wave_filename[32];
//bool wav_2nd_time = false;
int wav_fcount = 1;
bool dsp_active = false;
const int cbl = 50; // circular buffer length
String cb[cbl];     // circular buffer to store SD file name
uint32_t cb_sz[cbl];// circular buffer to store SD file size
uint32_t ct2 = 0;
uint32_t pt3 = 0;
bool pcf_active = false;

Audio audio(false, 3, I2S_NUM_1); // change default i2s port number
WiFiMulti wifiMulti;
#ifdef _Adafruit_SSD1306_H_
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
#ifdef _Adafruit_SH110X_H_
Adafruit_SH1106G oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#endif
RDA5807 radio;
Adafruit_PCF8574 pcf;

String ssid =     "ssid";      // WiFi 1 , set your wifi station
String password = "password";  // set your password
String ssid2 =     "ssid2";     // WiFi 2, optional
String password2 = "password2"; // set your password

//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

bool v_stream = false;

const char* STREAM_HEADER = 
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
  "\r\n";

const char* FRAME_HEADER = 
  "--frame\r\n"
  "Content-Type: image/jpeg\r\n"
  "Content-Length: %d\r\n"
  "\r\n";

struct tm *tm;
int d_mon ;
int d_mday ;
int d_hour ;
int d_min ;
int d_sec ;
int d_wday ;
int d_year ;
static const char *weekStr[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"}; //3文字の英字

int  vol;    // DSP volume
int  lastvol;
int  stnIdx;
int  laststnIdx;
int  stnFreq[] = {8040, 8250, 8520, 9040, 9150, 7620, 7810, 7860}; // frequency of radio station
String  stnName[] = {"AirG", "NW", "NHK", "STV", "HBC", "sanka", "karos", "nosut"}; // name of radio station max 5 char
//                      0      1     2      3      4       5        6       7
bool bassOnOff = false;
bool vol_ok = true;
bool stn_ok = true;
bool p_onoff_req = false;
bool p_on = false;
int volume,lastVolume;   // inet volume 

float lastfreq;
struct elm {  // program
   int stime; // strat time(min)
   int fidx;  // frequency table index
   int duration; // length min
   int volstep; // volume
   int poweroff; // if 1, power off after duration
   int scheduled; // if 1, schedule done for logic
};
struct elm entity[7][MAXSCEDIDX + 1] = {
{{390,1,59,2,1,0},{540,6,59,1,0,0},{600,0,59,1,0,0},{660,3,119,1,0,0},{780,1,59,1,0,0},{840,0,59,1,0,0},{900,6,59,1,0,0},{1140,3,119,1,0,0},{1410,0,29,1,1,0}}, // sun
{{390,4,59,2,1,0},{480,3,119,1,0,0},{600,6,59,1,0,0},{720,2,119,1,0,0},{840,1,119,1,0,0},{0,0,0,0,0,0},{1020,1,119,1,0,0},{1200,6,89,1,0,0},{1410,0,29,1,1,0}}, // mon
{{390,4,59,2,1,0},{480,3,119,1,0,0},{720,2,89,1,0,0},{840,1,119,1,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{1020,1,119,1,0,0},{1200,0,89,1,0,0},{1410,0,29,1,1,0}}, // tue
{{390,4,59,2,1,0},{480,3,119,1,0,0},{720,2,89,1,0,0},{840,1,119,1,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{1020,1,119,1,0,0},{1200,0,89,1,0,0},{1410,0,29,1,1,0}}, // wed
{{390,4,59,2,1,0},{480,3,119,1,0,0},{600,6,59,1,0,0},{720,2,119,4,0,0},{840,1,119,1,0,0},{960,1,59,1,0,0},{1080,1,119,1,0,0},{1200,0,89,1,0,0},{1290,3,59,1,1,0}}, // thu
{{390,4,59,2,1,0},{480,3,119,1,0,0},{660,0,59,1,0,0},{720,2,119,1,0,0},{840,6,119,1,0,0},{0,0,0,0,0,0},{1080,1,119,1,0,0},{1200,1,89,1,0,0},{1290,3,59,1,1,1}}, // fri
{{390,0,29,2,0,0},{420,2,119,1,0,0},{540,2,110,1,0,0},{720,2,119,1,0,0},{840,2,119,1,0,0},{960,2,119,1,0,0},{1080,4,59,1,0,0},{1140,3,119,1,0,0},{1260,0,89,1,1,0}}  // sat
};
//struct elm rom_entity[7][MAXSCEDIDX + 1];
int last_d_min = 99;
int currIdx = 99;
int pofftm_h = 0;
int pofftm_m = 0;
const char* ntpServer = "ntp.nict.jp";
const long  gmtOffset_sec = 32400;
const int   daylightOffset_sec = 0;

WebServer server(80);  // port 80(default)

// Operation by server
int s_srv = 1;
int a_srv = 1;
int b_srv = 1;
//char titlebuf[166];
//char rstr[128];
//char stnurl[128];  // current internet station url
String msg = "";
int stoken = 0;      // server token, count up 
int recording = 0;

Preferences preferences; // Permanent data

struct WavHeader_Struct {
  //   RIFF Section
  char RIFFSectionID[4];  // Letters "RIFF"
  uint32_t Size;          // Size of entire file 
  char RiffFormat[4];     // Letters "WAVE"

  //   Format Section
  char FormatSectionID[4];  // letters "fmt"
  uint32_t FormatSize;      // Size of format section
  uint16_t FormatID;        // 1=uncompressed PCM
  uint16_t NumChannels;     // 1=mono,2=stereo
  uint32_t SampleRate;      // 44100, 32000, 16000, 8000 etc.
  uint32_t ByteRate;        // =SampleRate * Channels * (BitsPerSample/8)
  uint16_t BlockAlign;      // =Channels * (BitsPerSample/8)
  uint16_t BitsPerSample;   // 8,16,24 or 32

  // Data Section
  char DataSectionID[4];  // The letters "data"
  uint32_t DataSize;      // Size of the data that follows
} WavHeader;

File WavFile;                                     // SD card directory

void generate_wav_header(uint8_t *wav_header, uint32_t wav_size, uint32_t sample_rate)
{
  // See this for reference: http://soundfile.sapp.org/doc/WaveFormat/
  uint32_t file_size = wav_size + WAV_HEADER_SIZE - 8;
  uint32_t byte_rate = SAMPLE_RATE * SAMPLE_BITS / 8;
  const uint8_t set_wav_header[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // Format
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
    0x01, 0x00, // AudioFormat (1 for PCM)
    //0x01, 0x00, // NumChannels (1 channel)
    0x02, 0x00, // NumChannels (2 channel)
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    //0x02, 0x00, // BlockAlign mono
    0x04, 0x00, // BlockAlign stereo
    0x10, 0x00, // BitsPerSample (16 bits)
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  const uint8_t set_wav_header_m[] = {
    'R', 'I', 'F', 'F', // ChunkID
    file_size, file_size >> 8, file_size >> 16, file_size >> 24, // ChunkSize
    'W', 'A', 'V', 'E', // Format
    'f', 'm', 't', ' ', // Subchunk1ID
    0x10, 0x00, 0x00, 0x00, // Subchunk1Size (16 for PCM)
    0x01, 0x00, // AudioFormat (1 for PCM)
    0x01, 0x00, // NumChannels (1 channel)
    sample_rate, sample_rate >> 8, sample_rate >> 16, sample_rate >> 24, // SampleRate
    byte_rate, byte_rate >> 8, byte_rate >> 16, byte_rate >> 24, // ByteRate
    0x02, 0x00, // BlockAlign
    0x10, 0x00, // BitsPerSample (16 bits)
    'd', 'a', 't', 'a', // Subchunk2ID
    wav_size, wav_size >> 8, wav_size >> 16, wav_size >> 24, // Subchunk2Size
  };
  if (REC_on) 
    memcpy(wav_header, set_wav_header, sizeof(set_wav_header));
  else
    memcpy(wav_header, set_wav_header_m, sizeof(set_wav_header_m));
}

int i2s_install(std::string type) {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config_dsp = { // for DSP radio
    .mode = i2s_mode_t(I2S_MODE_SLAVE | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // R-chan, L-chan
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S), //I2S Philips standard
    .intr_alloc_flags = 0, //
    .dma_buf_count = I2S_DMA_BUFFER,   // #### ex. 52
    .dma_buf_len = 1024,
    .use_apll = false
  };
  const i2s_config_t i2s_config_mic = { // for MIC
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX| I2S_MODE_PDM),
    .sample_rate = SAMPLE_RATE_MIC,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S ,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear   = false,
    .fixed_mclk           = 0  
  };
  const i2s_config_t i2s_config_dac = {  // for DAC out
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 32000U,  // Note, this will be changed later
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,  // high interrupt priority
    .dma_buf_count = 64,                       // 64 buffers
    .dma_buf_len = 64,                         // 64 bytes per buffer
    .use_apll = 0,
    .tx_desc_auto_clear = true,
    .fixed_mclk = -1                           
  };// --> NOT USED
  int erResult;
  if (type=="DSP") {
    erResult = i2s_driver_install(I2S_NUM_0, &i2s_config_dsp, 0, NULL);
  } else if(type=="MIC") {
    erResult = i2s_driver_install(I2S_NUM_0, &i2s_config_mic, 0, NULL);  // PDM is only num_0 OK
  } else {
    erResult = i2s_driver_install(I2S_NUM_0, &i2s_config_dac, 0, NULL); //  not used
  }
  if (erResult!=ESP_OK) Serial.printf("i2s intall %s err(%d)\n", type, erResult);
  return(erResult);
}

int i2s_setpin(std::string type) {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config_dsp = { // from DSP radio
    .bck_io_num = I2S_BCLK_S,
    .ws_io_num = I2S_LRC_S,
    .data_out_num = -1,
    .data_in_num = I2S_DIN_S
  };
  const i2s_pin_config_t pin_config_mic = { // from MIC
    .bck_io_num = I2S_PIN_NO_CHANGE, //  clock 
    .ws_io_num = 42,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = 41 //  Data
  };
  const i2s_pin_config_t pin_config_dac = {  // to DAC out
    .bck_io_num = I2S_BCLK_A,           //  clock 
    .ws_io_num = I2S_LRC_A,             //  Word select
    .data_out_num = I2S_DOUT_A,         //  Data out
    .data_in_num = I2S_PIN_NO_CHANGE    //       --> NOT USED
  };
  int erResult;
  if (type=="DSP") {
    erResult = i2s_set_pin(I2S_NUM_0, &pin_config_dsp);
  } else if (type=="MIC") {
    erResult = i2s_set_pin(I2S_NUM_0, &pin_config_mic);
  }  else {
    erResult = i2s_set_pin(I2S_NUM_0, &pin_config_dac); // not used
  }
  if (erResult!=ESP_OK) Serial.printf("i2s set pin %s err(%d)\n",type, erResult);
  return(erResult);
}

int split(String data, char delimiter, String *dst){
  int index = 0;
  int arraySize = (sizeof(data))/sizeof((data[0]));
  int datalength = data.length();
  
  for(int i = 0; i < datalength; i++){
    char tmp = data.charAt(i);
    if( tmp == delimiter ){
      index++;
      if( index > (arraySize - 1)) return -1;
    }
    else dst[index] += tmp;
  }
  return (index + 1);
}

int dayofWeek(String dow) {
  dow.trim();
  //Serial.print("dow:");
  //Serial.println(dow);
  if (dow.equals("Sun")) return 0; 
  else if (dow.equals("Mon")) return 1;
  else if (dow.equals("Tue")) return 2;
  else if (dow.equals("Wed")) return 3;
  else if (dow.equals("Thu")) return 4;
  else if (dow.equals("Fri")) return 5;
  else if (dow.equals("Sat")) return 6;
  else return 9;
}

int setWeeksced(String val1){
  String instr[12] = {"\n"};
  String instr2[8] = {"\n"};
  String instr3[4] = {"\n"};
  int ix = split(val1,';',instr);
  if (ix != 11) {
    msg = "different number of arguments.";
    return 4;
  } else {
    //msg = "arguments. ok.";
    int down = dayofWeek(instr[0]);
    if (down > 6) { msg = "invalid day of week."; return 4;}
    else {
      // normal process
      msg = "normal process.";
      instr[0].trim();
      Serial.println(instr[0]);
      for(int j = 0; j <= MAXSCEDIDX; j++) {
        instr[j+1].trim();
        msg = "normal process 2.";
        //Serial.println(instr[j+1]);
        String val2 = instr[j+1];
        ix = split(val2,',',instr2);
        if (ix != 5) { 
            msg = "different number of  2nd level arguments.";
            return 4;
        } else {
            //for(int i = 0; i < 5; i++) {
              msg = "OK! Processing.";
              //Serial.println(instr2[i]);
              val2 = instr2[0];
              ix = split(val2,':',instr3);
              if (ix != 2) {
                msg = "different number of  3rd level arguments.";
                return 4;
              }
              instr3[0].trim();
              instr3[1].trim();
              entity[down][j].stime = instr3[0].toInt() * 60 + instr3[1].toInt();
              instr3[0] = "";
              instr3[1] = "";
              instr2[0] = "";

              entity[down][j].fidx = instr2[1].toInt();
              instr2[1] = "";
              entity[down][j].duration = instr2[2].toInt();
              instr2[2] = "";
              entity[down][j].volstep = instr2[3].toInt();
              instr2[3] = "";
              entity[down][j].poweroff = instr2[4].toInt();
              instr2[4] = "";
              preferences.putString(weekStr[down], val1);  // save permanently              
            //}
        }        
      }
      msg = "OK! Done.";
      return 0;
    }
  } 
}

void handleRoot(void)
{
    String html;
    String val1;
    String val2;
    String val3;
    String val4;
    String val5;
    String val6;
    String html_btn1 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"record\"  value=\"Recording_Function\" class=\"btn\"></div></p>";
    String html_p1; 
    char htstr[180];
    char stnno[4];
    Serial.println("web received");
    if (server.method() == HTTP_POST) { // submitted with string
      val1 = server.arg("daysced");
      val2 = server.arg("vup");
      val3 = server.arg("vdown");
      val4 = server.arg("stnup");
      val5 = server.arg("stndown");
      val6 = server.arg("pwonoff");
      if (val2.length() != 0) {
        Serial.println("vup");
        vol_setting(); 
        msg = "control: vup";
      }
      else if (val3.length() != 0) {
        Serial.println("vdown");
        vol_setting_2(); 
        msg = "control: vdown";
      }
      else if (val4.length() != 0) {
        Serial.println("stnup");
        station_setting(); 
        msg = "control: stnup";
      }
      else if (val5.length() != 0) {
        Serial.println("stndown");
        station_setting_2(); 
        msg = "control: stndown";
      }
      else if (val6.length() != 0) {
        Serial.println("pwonoff");
        power_onoff_setting(); 
        msg = "control: pwonoff";
      }
      else {
        if (val1.length() == 0) {
          msg ="no input.";
        } else {
          int rc = setWeeksced(val1);
        }
      }
    }
    html = "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>DSP radio Weekly schedule</title>";
    html += "</head><body><p><h3>DSP Radio Schedule and Recording (ver 0.76)</p></h3>"; // VERSION_NO
    html += "<style>.lay_i input:first-of-type{margin-right: 20px;}</style>";
    html += "<style>.btn {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #cbe8fa; cursor: pointer;}</style>";
    html += "<style>.btn_y {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #ffff8a; cursor: pointer;}</style>";
    html += "<style>.btn_g {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #99ff99; cursor: pointer;}</style>";

    html += "<form action=\"/rec\" method=\"get\">" + html_btn1 + "</form>";
    html += "<form action=\"\" method=\"post\">";
    html += "<input type=\"hidden\" name=\"stoken\" value=\"";
    stoken += 1;
    html += stoken;
    html += "\">"; 
    html += "<p>Select a day of the week, change it, then submit.</p>";
    html += "<p>";
    html += "<input type=\"text\" id=\"daysced\" name=\"daysced\" size=\"120\" value=\"\">";
    html += "</p><p><input type=\"submit\" value=\"submit\" class=\"btn\"></p></form>";
    html += "<p>" + msg + "</p>";
    html += "<p>Arguments of enrty: Start time(hour:min),Station(See below),Duration(min),Volume,Pweroff</p>";
    html += "<p>Station List: 0=" + stnName[0] + ",1=" + stnName[1] + ",2=" + stnName[2] + ",3=" + stnName[3] + ",4=" + stnName[4];
    html += ",5=" + stnName[5] + ",6=" + stnName[6] +  ",7=" + stnName[7] + "</p>";
    html += "<script>";
//    html += "let entity = [[[390,1,59,4,1],[540,6,59,2,0],[600,0,59,2,0],[660,3,119,2,0],[780,1,59,2,0],[840,0,59,2,0],[900,1,59,2,0],[1140,3,119,2,0],[1410,0,29,2,1]],";
//    html += "[[390,1,59,4,1],[540,6,59,2,0],[600,0,59,2,0],[660,3,119,2,0],[780,1,59,2,0],[840,0,59,2,0],[900,1,59,2,0],[1140,3,119,2,0],[1410,0,29,2,1]]]";
    html += "let entity = [";
    for (int i = 0; i < 7; i++){
      html += "[";
      for(int j = 0; j <= MAXSCEDIDX; j++) {
        sprintf(htstr,"['%d:%02d',%d,%d,%d,%d]",entity[i][j].stime / 60,entity[i][j].stime % 60,entity[i][j].fidx,entity[i][j].duration,entity[i][j].volstep,entity[i][j].poweroff);
        html += htstr;
        if (j != MAXSCEDIDX) html += ",";
      }
      html += "]";
      if (i != 6) html += ",";
    }
    html += "];";
    html += "let week = [\"Sun\",\"Mon\",\"Tue\",\"Wed\",\"Thu\",\"Fri\",\"Sat\"];";
    html += "document.write('<table id=\"tbl\" border=\"1\" style=\"border-collapse: collapse\">');";
    html += "for (let i = 0; i < 7; i++){";
    html += "let wstr ='';";
    html += "wstr ='<tr>' + '<td>' + '<input type=\"radio\" name=\"week\" value=\"\" onclick=\"setinput(' + i + ')\">' + '</td>' + '<td>' + week[i] + '</td>';";
    html += "document.write(wstr);";
    html += "for (let j = 0; j < 9; j++){";
    html += "document.write('<td>');";
    html += "document.write(entity[i][j]);";
    html += "document.write('</td>');}";
    html += "document.write('</tr>');";
    html += "}";
    html += "document.write('</table>');";
    html += "function setinput(trnum) {";
    html += "var input = document.getElementById(\"daysced\");";
    html += "var table = document.getElementById(\"tbl\");";
    html += "var cells = table.rows[trnum].cells;";
    html += "let istr = '';";
    html += "for (let j = 1; j <= 10; j++){";
    html += "istr = istr + cells[j].innerText + ';';";
    html += "}";
    html += "input.value = istr;";
    html += "}";
    html += "</script>";
    html += "<style>.lay_i input:first-of-type{margin-right: 20px;}</style>";
    html += "<style>.btn {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #cbe8fa; cursor: pointer;}</style>";
    html += "<p><form action=\"\" method=\"post\">";
    html += "<p>Control Functions</p>";
    html += "<p><div class=\"lay_i\"><input type=\"submit\" name=\"vup\"  value=\"volume up\" class=\"btn\"><input type=\"submit\" name=\"vdown\" value=\"volume down\" class=\"btn\"></div></p>";
    html += "<p><div class=\"lay_i\"><input type=\"submit\" name=\"stnup\"  value=\"station up\" class=\"btn\"><input type=\"submit\" name=\"stndown\" value=\"station down\" class=\"btn\"></div></p>";
    html += "<p><div class=\"lay_i\"><input type=\"submit\" name=\"pwonoff\"  value=\"pwr_on_off\" class=\"btn\"></div></p>";
    html += "</form></p></body>";
    html += "</html>";
    server.send(200, "text/html", html);
    Serial.println("web send response");
}
void handleRec(void)
{
  String html;
  String val1, val2, val3, val4, val5, val6, val7, val8, val9, val10, val11, val12;
  char ts[40];
  //const int cbl = 30; // circular buffer length
  //String cb[cbl];     // circular buffer to store SD file name
  uint32_t total_file_size = 0;
  bool no_refresh = false;

  bool responsed = false;
  String html_btn0 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"record_start\"  value=\"Start_DSP_Recording\" class=\"btn\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"rec_stop\" value=\"Stop_DSP_Recording\" class=\"btn\"></div></p>";
  String html_btn1 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"record_start\"  value=\"Start_DSP_Recording\" class=\"btn_g\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"rec_stop\" value=\"Stop_DSP_Recording\" class=\"btn\"></div></p>";
  String html_btn3 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"record_start\"  value=\"DSP_Recording_in_Progress\" class=\"btn_r\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"rec_stop\" value=\"Stop_DSP_recording\" class=\"btn\"></div></p>";
  String html_btn4 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"play_stop\"  value=\"Stop_Play\" class=\"btn\"><input type=\"submit\" name=\"forward_5min\"  value=\"Forward_5_min\" class=\"btn\"></div></p>";;
  String html_btn5 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"play_stop\"  value=\"Stop_Play\" class=\"btn_y\"><input type=\"submit\" name=\"forward_5min\"  value=\"Forward_5_min\" class=\"btn\"></div></p>";;
  String html_btn6 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"init_camera\"  value=\"Init_Camera\" class=\"btn\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"shoot_camera\" value=\"Shoot_Camera\" class=\"btn\"><input type=\"submit\" name=\"stream_camera\" value=\"Stream_Camera\" class=\"btn\"></div></p>";
  String html_btn7 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"init_camera\"  value=\"Init_Camera\" class=\"btn_r\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"shoot_camera\" value=\"Shoot_Camera\" class=\"btn\"><input type=\"submit\" name=\"stream_camera\" value=\"Stream_Camera\" class=\"btn\"></div></p>";
  String html_btn8 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"mic_rec_start\"  value=\"Start_MIC_Recording\" class=\"btn\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"mic_rec_stop\" value=\"Stop_MIC_Recording\" class=\"btn\"></div></p>";
  String html_btn9 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"mic_rec_start\"  value=\"Start_MIC_Recording\" class=\"btn_g\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"mic_rec_stop\" value=\"Stop_MIC_Recording\" class=\"btn\"></div></p>";
  String html_btn10 = "<p><div class=\"lay_i\"><input type=\"submit\" name=\"mic_rec_start\"  value=\"MIC_Recording_in_Progress\" class=\"btn_r\"><div class=\"triangle-right\"></div><input type=\"submit\" name=\"mic_rec_stop\" value=\"Stop_MIC_recording\" class=\"btn\"></div></p>";
  String html_p1, html_p2, html_p3; 
  html_p1 = html_btn0;
  html_p2 = html_btn4;
  html_p3 = html_btn8;
  Serial.println("web received(Rec)");
  val2 = server.arg("rec_stop");
  msg = "";
  if ((server.method() == HTTP_POST) && (!REC_on || val2.length() != 0)) { // submitted with string
    val1 = server.arg("record_start");
    val2 = server.arg("rec_stop");
    val3 = server.arg("play_stop");
    val4 = server.arg("stoken");
    val5 = server.arg("format");
    val6 = server.arg("status");
    val7 = server.arg("forward_5min");
    val8 = server.arg("init_camera");
    val9 = server.arg("shoot_camera");
    val10 = server.arg("stream_camera");
    val11 = server.arg("mic_rec_start");
    val12 = server.arg("mic_rec_stop");
    if (val4.length() != 0) { // server token
      Serial.print("stoken:");
      String s_stoken = server.arg("stoken");
      int t_stoken = s_stoken.toInt();
      Serial.println(s_stoken);
      msg = "stoken:" + s_stoken;
      if (stoken > t_stoken) {
        Serial.println("redirect-rec");
        msg = "Post converted to Get";
        responsed = true;
        server.send(303, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/rec\"></head></html>");
        no_refresh = true;
      }
    } else{ ;
    }
  }
  if (!responsed) {
    if (val2.length() != 0) { // rec stop request   
      Serial.println("rec stop");
      if (REC_on) { // Is recoding active ?
        pofftm_h = (d_hour * 60 + d_min + 1) / 60;
        pofftm_m = (d_hour * 60 + d_min + 1) % 60;
        sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
        Serial.println(ts); 
        msg = "control: rec stop scheduled, wait a few minutes";
        REC_on_no_poff = true;
      } else {
        msg = "control: rec stop ignored";        
      }
      no_refresh = true;
    } else 
    if (val3.length() != 0) { // play stop req   
      Serial.println("play_stop");
      msg = "control: play stop";
      stop_read = true;    // Stop read
      no_refresh = true;
    } else
    if (val1.length() != 0) {
        Serial.println("record");
        if (!REC_on && stop_read) {
          total_recorded_size = 0;
          last_blk = 0;
          estimated_recorded_size = MAX_RECORD_TIME * SAMPLE_RATE * SAMPLE_BITS * 60 * 2 / 8;  // MAX_RECORD_TIME min
          pofftm_h = (d_hour * 60 + d_min + MAX_RECORD_TIME) / 60;  // auto stop after MAX_RECORD_TIME min
          pofftm_m = (d_hour * 60 + d_min + MAX_RECORD_TIME) % 60;
          sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
          Serial.println(ts); 
          REC_on_no_poff = true;
          esp_err_t err = i2s_install("DSP");
          i2s_setpin("DSP");
          if (err != ESP_OK) {
            Serial.println("Failed to initialize I2S!");
            I2S_err = true;
          }
          if(!I2S_err && !SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")){ // SD mount
            Serial.println("Failed to mount SD Card!");
            I2S_err = true;
            i2s_stop(I2S_NUM_0);  // nomore DSP I2S now
          }
          if (!I2S_err) {
            REC_on = true; // start REC ok
            REC_on_no_poff = true;
            msg = "control: record";
          } else {
            msg = "control: record err";
          }
        } else {
          msg = "control: record ignored";
        }
    } else 
    if (val6.length() != 0) {
      msg = "now recording is active";
      no_refresh = true;
    } else
    if (val5.length() != 0){
      msg = "invalid format";
      no_refresh = true;
    }  else
    if (val7.length() != 0) { // forward
      Serial.println("web: forward.");
      msg = "control: forward";
      no_refresh = true;
      if (!stop_read) { // is playing ?
        uint32_t f_size = audio.getFileSize(); // in bytes
        uint32_t f_pos = audio.getFilePos();
        f_pos += record_size * 5 * 60 ; // 5 min
        if (f_pos < f_size)  {
           audio.setFilePos(f_pos);
           Serial.printf("forward to: %d\n", f_pos);
        } else Serial.println("web: forward over file size.");
      }
    } else
    if (val9.length() != 0) { // shoot
      Serial.println("web: shoot.");
      msg = "control: shoot";
      no_refresh = true;
      if (camera_ok && !REC_on && !MIC_rec_on) {
        msg = "control: shoot";
        shoot_s = true;
      } else {
        msg = "err: camera not initialized or SD busy";
      }
    } else
    if (val8.length() != 0) { // init camera
      Serial.println("web: init camera.");
      msg = "control: init camera";
      no_refresh = true;
      if (!camera_ok) {
        bool init_ok =set_camera(FRAMESIZE_XGA);
        if (init_ok) {
          msg = "control: init camera ok";
          Serial.println("web: init camera ok.");
        }    
      }
    } else
    if (val10.length() != 0) { // stream
      //responsed = true;
      Serial.println("web: stream.");
      msg = "control: stream";
      no_refresh = true;
      if (camera_ok) {
        Serial.println("redirect-stream");
        msg = "Stream req redirect";
        server.send(303, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/stream\"></head></html>");
        responsed = true;
      } else {
        msg = "err: camera not initialized";
      }
    } else
    if(val11 != 0) { // MIC rec start
      Serial.println("web: MIC rec start.");
      msg = "control: MIC rec start";
        if (!REC_on && stop_read && !MIC_rec_on) {
          total_recorded_size = 0;
          last_blk = 0;
          estimated_recorded_size = MAX_RECORD_TIME * SAMPLE_RATE * SAMPLE_BITS * 60 / 8;  // MAX_RECORD_TIME min
          pofftm_h = (d_hour * 60 + d_min + MAX_RECORD_TIME) / 60;  // auto stop after MAX_RECORD_TIME min
          pofftm_m = (d_hour * 60 + d_min + MAX_RECORD_TIME) % 60;
          sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
          Serial.println(ts); 
          REC_on_no_poff = true;

          esp_err_t err = i2s_install("MIC");
          //I2S.setAllPins(-1, 42, 41, -1, -1); // XIAO ESP32S3 SENSE PIN USAGE
          i2s_setpin("MIC");         
          if (err != ESP_OK) {
            Serial.println("Failed to initialize I2S!");
            I2S_err = true;
          } else 
            Serial.println("mic i2s ok"); 
          if(!I2S_err && !SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")){ // SD mount
            Serial.println("Failed to mount SD Card!");
            I2S_err = true;
            i2s_driver_uninstall(I2S_NUM_0);  // no more DSP I2S now
          }
          if (!I2S_err) {
            MIC_rec_on = true; // start REC ok
            REC_on_no_poff = true;
            msg = "control: record";
            radio.powerDown();
            dsp_active = false;
            Serial.println("mic rec on"); 
          } else {
            msg = "control: record err";
          }
        } else {
          msg = "control: record ignored";
        }
            
    } else
    if (val12 != 0) { // MIC rec stop
      Serial.println("web: MIC rec stop.");
      msg = "control: MIC rec stop";
      if (MIC_rec_on) { // Is recoding active ?
        pofftm_h = (d_hour * 60 + d_min + 1) / 60;
        pofftm_m = (d_hour * 60 + d_min + 1) % 60;
        sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
        Serial.println(ts); 
        msg = "control: rec stop scheduled, wait a few minutes";
        REC_on_no_poff = true;
      } else {
        msg = "control: rec stop ignored";        
      }
      no_refresh = true;
    }
    else {
        //nop
    }     
  }
  if (REC_on_no_poff) {
    if (MIC_rec_on) {html_p1 = html_btn0 + html_btn10;} else {html_p1 = html_btn3 + html_btn8;}
  } else {
    if (MIC_rec_on) {html_p1 = html_btn0 + html_btn10;} else {html_p1 = html_btn0 + html_btn8;}
  }
  if (camera_ok) html_p1 += html_btn7; else html_p1 += html_btn6;
  if (stop_read) html_p2 = html_btn4; else html_p2 = html_btn5;
  if (!responsed) {
    html = "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>DSP Radio Recording</title>";
    html += "</head><body><p><h3>Recording and Playing&nbsp;&nbsp;(experimental)</h3>&nbsp;&nbsp;<a href=\"/\">Back</a></p><form action=\"\" method=\"post\">";
    //html += "<style>.lay_i input:first-of-type{margin-right: 20px;}</style>";
    html += "<style>.lay_i input {margin-right: 20px;}</style>";
    html += "<style>.btn {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #cbe8fa; cursor: pointer;}</style>";
    html += "<style>.btn_y {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #ffff8a; cursor: pointer;}</style>";
    html += "<style>.btn_g {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #99ff99; cursor: pointer;}</style>";
    html += "<style>.btn_r {width: 300px; padding: 10px; box-sizing: border-box; border: 1px solid #68779a; background: #FFA5A5; cursor: pointer;}</style>";
    html += "<style>.triangle-right {display: inline-block; border-style: solid; border-width: 8px 0 8px 18px; border-color: transparent transparent transparent #000; margin-right: 16px; position:relative; top: 4px;}</style>";
    html += html_p1;
    html += "<input type=\"hidden\" name=\"stoken\" value=\"";
    stoken += 1;
    html += stoken;
    html += "\">"; 
    html += "<p><form action=\"/wavf\" method=\"post\">";
    //html += "<p><div class=\"lay_i\"><input type=\"submit\" name=\"play_stop\"  value=\"STOP PLAY\" class=\"btn\"></div></p>";
    html += html_p2;
    html += "</form></p>";
    html += "<p>Response: " + msg + "</p>";
    html += "<p><h3>Files:</h3>&nbsp;&nbsp;Press 'file name link' to play</p>";
    if(!REC_on && !MIC_rec_on) {
      if (!SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")){ // SD mount
         Serial.println("Failed to mount SD Card!");
         html += "<p>Failed to mount SD Card!</p>";
      } else {
        File root = SD.open("/");
        uint64_t tb = SD.totalBytes();
        uint32_t tbi = tb/(1024*1024); // MB
        uint64_t ub = SD.usedBytes();
        uint32_t ubi = ub/(1024*1024); // MB
        String sdinfo_tb(tbi);
        String sdinfo_ub(ubi);
        String sdinfo_ra( ( (tbi-ubi) * 1024) / ( ( ( (SAMPLE_RATE * SAMPLE_BITS * CHAN_NUM / 8 ) /1024  ) * 60) )  );
        html += "<p>&nbsp;&nbsp;total size(MB):&nbsp;&nbsp;" + sdinfo_tb; 
        html += "&nbsp;&nbsp;used size(MB):&nbsp;&nbsp;" + sdinfo_ub;
        html += "&nbsp;&nbsp; remaining amount(minutes):&nbsp;&nbsp;" + sdinfo_ra + "</p>";
        bool isDir = false;
        String fname;
        int cbix = 0; // circular buffer index
        int fcnt = 0;
        while (true) {
          String filename = root.getNextFileName(&isDir);
          if (filename == "") break; // nomore files
          if (!isDir) { // not directory
            if (filename.length() > 4) {
              fname = filename.substring(1,28);
              cb[cbix] = fname;
              cbix++;
              fcnt++;
              if (cbix >= cbl) cbix = 0; // reset index
            }
          }
        }
        String sdinfo_fi(fcnt);
        String sdinfo_lf(cbl);
        Serial.printf("filecount: %d\n", fcnt);
        if (fcnt > 0) { //Are there Files?
          html += "<p>&nbsp;&nbsp;total " + sdinfo_fi + " files&nbsp;&nbsp;"; 
          html += "&nbsp;&nbsp;(max listed " + sdinfo_lf + " files)<p>";
          int rdix = 0;
          for (int i = 0; i < cbl && i < fcnt; i++) {
            if (rdix >=  cbl) rdix = 0;
            if (cb[rdix].length()==25 && cb[rdix].substring(22,25)=="jpg") 
              html += "<p><a href='/jpgf?fname=" + cb[rdix] + "'>" + cb[rdix] + "</a>";
            else 
              html += "<p><a href='/wavf?fname=" + cb[rdix] + "'>" + cb[rdix] + "</a>";
            String ts = "/" + cb[rdix];
            char tstr[32] = {'\n'};
            uint32_t fsize;
            File wfile;
            ts.toCharArray(tstr, 29);
            if (!no_refresh) { // aboid to read from SD
              wfile = SD.open((char *)tstr, FILE_READ);
              fsize = wfile.size();
              cb_sz[rdix] = fsize;  // save it
            } else {
              fsize = cb_sz[rdix];  // restore from memory
            }
            total_file_size = total_file_size + fsize/1024;
            int fminutes = fsize / (SAMPLE_RATE * SAMPLE_BITS * CHAN_NUM / 8) / 60 + 1;
            ts = String(fsize/1024);
            html += "&nbsp;&nbsp;&nbsp;&nbsp;file size(KB):&nbsp;&nbsp;" + ts + "&nbsp;&nbsp;";
            ts = String(fminutes);
            html += "&nbsp;&nbsp;length(minutes):&nbsp;&nbsp;" + ts + "</p>";
            if (!no_refresh) wfile.close();
            rdix++;
          }
        } else {
          html += "<p>no files.</p>";
        }   
        root.close();
      }
    } else {
      html += "Recording in progress. If you want to stop recording, press Stop_recording button,<br>";
      html += "and wait a few minutes.<br>";
    }
    html += "</body>";
    html += "</html>";
    Serial.printf("Total file size(KB): %d\n", total_file_size);
    server.send(200, "text/html", html);
  }
  Serial.println("web send response(Rec)");
}

void handleWavf() {
  String html;
  String val1;
  Serial.println("Play start");
  val1 = server.arg(0);
  WiFiClient client = server.client();
  if (!client.connected()) {
    Serial.println("Client disconnected");
    return;
  }
  char tstr[101] = {'/n'};
  val1 = "/" + val1;
  val1.toCharArray(tstr, val1.length() + 1);
  Serial.println(tstr);
  if (!REC_on && !MIC_rec_on) {
    WavFile = SD.open((char *)tstr, FILE_READ);  // Open the wav file
    if (WavFile == false)
      Serial.println("Could not open wavfile");
    else {
      if (memcmp(tstr, "/inet_", 6) == 0) { // inet url 
        char inet_url[100] = {'\n'};
        int p = 0;
        while ( WavFile.available() && (p <= 100) )
        {
          char bc[2] = {'\n'};
          WavFile.read((uint8_t*)bc, 1);
          if( (bc[0] == 0x0d) || (bc[0] == 0x0a) || (bc[0] == 0x20) ) break;
          inet_url[p] = bc[0];
          p++;
        }
        if ( (p > 10) && (p <= 99) ) {  // may be url
          Serial.printf("inet detected: %s\n", inet_url);
          bool conn_ok = audio.connecttohost(inet_url);
          if (conn_ok) {
            pcf.digitalWrite(6, LOW);
            stop_read = false; // ok, start it
          } else {
            stop_read = true; // cannot connect
          }
        }
        WavFile.close();
      }   else   {
        WavFile.read((byte*)&WavHeader, 44);                    // Read  WAV header, first 44 bytes of the file.
        int rc = DumpWAVHeader(&WavHeader);                     // confirm  header data
        WavFile.close();  //
        if (rc <= 1) {  // wav or mp3 ?
          i2s_start(I2S_NUM_1); // start port of I2S Audio #### 0 -> 1 (0.74)
          delay(100);
          bool cc = audio.connecttoFS(SD, tstr); // play this file in the SD
          //bool  cc = false;
          if (cc) {
            pcf.digitalWrite(6, LOW);
            stop_read = false; // ok, start it  
          } else {
            Serial.printf("connectFS fail");
            stop_read = true; // stop it  
          }
        }
      }
    }
  } else {
    msg = "Now recording is active";
    Serial.println("redirect-invalid-status");
    server.send(303, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/rec?status=invalid\"></head></html>");
  }
  if (!stop_read) {
    //server.send(200, "text/plain", "Ok Play start. To stop Play, press <a href=\"/rec\">backward</a>, then press Stop_Play button on the screen.");
    server.send(200, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"></head><body>Ok Play start. To stop Play, press &nbsp;<a href=\"/rec\">backward</a>, then press Stop_Play button on the screen.</body></html>");
    Serial.println("Play continue");
  } else {
    msg = "invalid format";
    Serial.println("redirect-invalid-format");
    // never use 301 redirect, it's parmanent. 
    server.send(303, "text/html", "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url=/rec?format=invalid\"></head></html>");
  }
}

void handleJPGStream() {
  WiFiClient client = server.client();
  if (!client.connected()) {
    Serial.println("Client disconnected");
    return;
  }
  Serial.println("Stream start");
  if (camera_ok) {
    v_stream = true;
    camera_fb_t * fb = NULL;
    sensor_t *s = esp_camera_sensor_get();
    //s->set_framesize(s, FRAMESIZE_SVGA);
    s->set_framesize(s, FRAMESIZE_XGA);

    delay(100);

    client.print(STREAM_HEADER);
    while (client.connected()) {
      //
      fb = esp_camera_fb_get();
      if (!fb) {
        Serial.println("Camera capture failed");
        return;
      }
      client.printf(FRAME_HEADER, fb->len);
      client.write(fb->buf, fb->len);  //send JPEG data
      esp_camera_fb_return(fb);
      delay(66); // 15 fps
    }
    //s->set_framesize(s, FRAMESIZE_QQVGA);
    v_stream = false;
  } else {
    server.send(200, "text/plain", "Camera is not initialized.");
  }
  Serial.println("Stream end");
}
void handleJpgf() {
  String val1;
  const char* JPG_HEADER = 
  "HTTP/1.1 200 OK\r\n"
  "Content-Type: image/jpeg\r\n"
  "Content-Length: %d\r\n"
  "\r\n";

  //byte readArray[40000]; // 40k  for SVGA
  Serial.println("Photo start");
  val1 = server.arg(0);
  val1 = "/" + val1;
  //Serial.println(val1);
  WiFiClient client = server.client();
  if (!client.connected()) {
    Serial.println("Client disconnected");
    return;
  }
  char tstr[32] = {'/n'};
  val1.toCharArray(tstr, 29);
  Serial.println(tstr);
  File jpgfile = SD.open((char *)tstr, FILE_READ);
  long fsize = 0;
  if (jpgfile) { // open ok
    fsize = jpgfile.size();
    Serial.printf("filesize: %d\n", fsize);
    long i = 0;
    if (fsize < 300000) { // SVGA:40000, XGA:300000
      while(jpgfile.available()) {
        byte rb = jpgfile.read();
        //readArray[i++] = rb;
        //(uint8_t *)(jpg_buffer + i) = rb;
        memset((uint8_t *)(jpg_buffer + i), rb ,1);
        i++;
      }
      Serial.printf("readsize: %d\n", i);
      if (i!=0) {
        client.printf(JPG_HEADER, i);
        int j = i / 1000;
        int k = i % 1000;
        int m = 0;
        for (int n = 0; n < j ; n++) {
          client.write((uint8_t *)jpg_buffer + m, 1000);
          m = m + 1000;
        }
        if (k > 0) client.write((uint8_t *)jpg_buffer + m, k);
        client.print("\r\n");
      } else {
        server.send(200, "text/plain", "file size = 0");
      }

    } else {
      Serial.println("File size is too big");
    }
    jpgfile.close();
  } else {
    Serial.println("file open err");
    server.send(200, "text/plain", "file open err");
  }
  Serial.println("Photo end");
}
void handleNotFound(void)
{
  server.send(404, "text/plain", "Not Found.");
}

// Save pictures to SD card
void photo_save(const char * fileName) {
  camera_fb_t *fb;
  // Take a photo ,  warm-up exercises are necessary to get clear photo
  for (int i = 1; i < 6; i++) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Failed to get camera frame buffer");
      return;
    }
    esp_camera_fb_return(fb);
    delay(66);
  }
  // use last 1 shoot
  fb = esp_camera_fb_get();
  // Save photo to file
  writeFile(SD, fileName, fb->buf, fb->len);
  
  // Release image buffer
  esp_camera_fb_return(fb);

  Serial.println("Photo saved to file");
}

// SD card write file
void writeFile(fs::FS &fs, const char * path, uint8_t * data, size_t len){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.write(data, len) == len){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

/*bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;
   tft.drawRGBBitmap(x, y, bitmap, w, h);
   // Return 1 to decode next block
   return 1;
}*/

bool set_camera(framesize_t f_size) {
  // Camera pinout
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = f_size;
  //config.frame_size = FRAMESIZE_240X240;
  //config.frame_size = FRAMESIZE_XGA;
  //config.frame_size = FRAMESIZE_QQVGA;  // 120x160
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      Serial.println("PSRAM found");
      config.jpeg_quality = 10;  //#### 12
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_XGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
    //config.frame_size = FRAMESIZE_QSXGA;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return(false);
  }
  Serial.println("Camera ready");
  sensor_t *s = esp_camera_sensor_get();
  //s->set_vflip(s, 0);
  //s->set_framesize(s, FRAMESIZE_SVGA);
  s->set_framesize(s, FRAMESIZE_XGA);
  camera_ok = true; // Camera initialization check passes
  return(true);
}

void SDCardInit() {
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);  // SD card chips select, must use GPIO 21 (ESP323 sense)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(REC_FREQUENCY);  // 10 - 24MHz
  if (!SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")) {
    Serial.println("Error talking to SD card!");
  } else sd_ok = true;
}
void setup()
{
  Serial.begin(115200);
  Serial.println("start");
  SDCardInit();

  //pinMode(1, INPUT_PULLUP);  // mode_setting
  //pinMode(3, INPUT_PULLUP);  // station_setting
  //pinMode(4, INPUT_PULLUP);  // power on_off
  //digitalWrite(1, HIGH);
  //digitalWrite(3, HIGH);
  //digitalWrite(4, HIGH);
  //attachInterrupt(1, vol_setting, FALLING); 
  //attachInterrupt(3, station_setting, FALLING); 
  //attachInterrupt(4, power_onoff_setting, FALLING); 

  Wire.setPins(PIN_SDA, PIN_SCL);  
  Wire.begin(); // 
  //Wire.setClock(400000);
  dsp_active = true;

  Wire.beginTransmission(0x11);
  Wire.write(0x04); // REG4
  Wire.write(0b10001000); // RDSIEN, De-emphasis 50μs
  Wire.write(0b01000000); // I2S Enabled
  Wire.endTransmission(); // stop transmitting
  Wire.beginTransmission(0x011);
  Wire.write(0x06); // REG6
  Wire.write(0b00000010); //  MASTER, DATA_SIGNED
  //Wire.write(0b00000000); //  MASTER, DATA_UNSIGNED #####2025/10/3
  //Wire.write(0b00010010); // SLAVE, DATA_SIGNED #####2025/10/3
  //Wire.write(0b10000000); // 48KBPS
  //Wire.write(0b01110000); // 44.1KBPS
  Wire.write(0b01100000); // 32KBPS
  //Wire.write(0b00000000); // 8KBPS
  //Wire.write(0b00110000); // 16KBPS
  Wire.endTransmission(); // stop transmitting

  //oled.begin(SH110X_SETCOMPINS, OLED_I2C_ADDRESS);
  oled.begin(OLED_I2C_ADDRESS, true);
  oled.clearDisplay();
  oled.setTextColor(1); // WHITE

  // Permanent data check
  preferences.begin("week_sced", false);
  for (int i = 0; i < 7; i++){
    String val1 = preferences.getString(weekStr[i],"");       
    if (val1 != "") {
      //Serial.println(val1);
      int rc = setWeeksced(val1);
    }
  }
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(ssid.c_str(), password.c_str());  
  wifiMulti.addAP(ssid2.c_str(), password2.c_str());
  wifiMulti.run();   // It may be connected to strong one
  
  while (true) {
    if(WiFi.status() == WL_CONNECTED){ break; }  // WiFi connect OK then next step
    Serial.println("WiFi Err");
    oled.setTextSize(2); // Draw 2X-scale text
    oled.setCursor(0, 0);
    oled.print("WiFi Err");
    oled.display();
    WiFi.disconnect(true);
    delay(5000);
    wifiMulti.run();
    delay(1000*300);  // Wait for Wifi ready
  }
  Serial.println("wifi start");
  wifisyncjst(); // refer time and day
  splash();

  radio.setup(); // Stats the receiver with default valuses. Normal operation
  delay(500);
  radio.setBand(2); //
  radio.setSpace(0); //
  delay(300);
  p_on = true;
  vol = preferences.getInt("vol_r", -1);
  if (vol < 0)  vol = 1;
  radio.setVolume(vol);
  lastvol=vol;
  stnIdx = preferences.getInt("stix", -1);
  if (stnIdx < 0)  stnIdx = 3;
  lastfreq = stnFreq[stnIdx];
  laststnIdx = stnIdx;
  radio.setFrequency(lastfreq);  // Tune on last
  // web server
  server.on("/", handleRoot);
  server.on("/wavf", handleWavf);
  server.on("/rec", handleRec);
  server.onNotFound(handleNotFound);
  server.on("/stream", HTTP_GET, handleJPGStream);
  server.on("/jpgf", HTTP_GET, handleJpgf);

  server.begin();
  Serial.print("IP = ");
  Serial.println(WiFi.localIP());
  //titlebuf[0] = 0;
  // PSRAM malloc for recording
  rec_buffer1 = (uint8_t *)ps_malloc(record_size);
  rec_buffer2 = (uint8_t *)ps_malloc(record_size);
  rec_buffer32k = (uint8_t *)ps_malloc(1024*32);
  jpg_buffer = (uint8_t *)ps_malloc(1024*300);  // SVGA:40k, XGA:300K
  if (rec_buffer1 != NULL && rec_buffer2 != NULL && rec_buffer32k != NULL && jpg_buffer != NULL) {
    memset(rec_buffer32k, 0, 1024*32); // 0 clear
  } else { 
    Serial.printf("malloc failed!\n");
    I2S_err = true;    
  }
  Serial.printf("Buffer: %d bytes\n", ESP.getPsramSize() - ESP.getFreePsram());
  SDCardInit();
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);  // use I2S_NUM_1 port
  volume = preferences.getInt("vol", -1);
  if (volume < 0) volume = 3;
  audio.setVolume(volume); // 0...21
  lastVolume = volume;
  i2s_stop(I2S_NUM_1); // stop I2S of audio because of noisy
  wav_fcount = preferences.getInt("wavf_no", -1);
  if (wav_fcount < 0) wav_fcount = 1;
  //pcf8574 init
  if (!pcf.begin(0x20, &Wire)) {
    Serial.println("Couldn't find PCF8574");
  } else {
    Serial.println("Find PCF8574");
    pcf_active = true;
    for (int p=0; p<6; p++) {  // 5 contact point switch
      pcf.pinMode(p, INPUT_PULLUP);
    }
    pcf.pinMode(6, OUTPUT);  // ADG884 control
    pcf.digitalWrite(6, HIGH); 
    delay(10);
  }
  Serial.println("Setup done");

}
void splash()
{
  IPAddress ipadr = WiFi.localIP();
  oled.setTextSize(2); // Draw 2X-scale text
  oled.setCursor(0, 0);
  oled.print("Clock");
  oled.setCursor(0, 15);
  oled.print("Radio_Rec");
  oled.display();
  delay(500);
  oled.setCursor(0, 30);
  oled.printf("IP:%d.%d", ipadr[2],ipadr[3]); // display last octet
  oled.setCursor(0, 45);
  oled.printf("V %.2f", VERSION_NO);
  oled.display();
  delay(1000);
}

void wifisyncjst() {
  //---------内蔵時計のJST同期--------
  // NTPサーバからJST取得
  int lcnt = 0;
  //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  configTzTime("JST-9", "ntp.nict.jp", "ntp.jst.mfeed.ad.jp");
  delay(500);
  // 内蔵時計の時刻がNTP時刻に合うまで待機
  while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) {
    delay(500);
    lcnt++;
    if (lcnt > 100) {
      Serial.println("time not sync within 50 sec");
      break;
    }
  }
}

void loop()
{
  char ts[80];
  float tf;
  char wave_filename_t[32];
  uint32_t pt2 = millis();
  if (pcf_active && ((pt2 - ct2) > 200) && !v_stream) { // check manual switch input
    for (int p=0; p<6; p++) {
      if (! pcf.digitalRead(p)) { // pressed ?
        switch(p) { 
          case 0:  //F
                vol_setting();
                break;
          case 1:  // B
                vol_setting_2(); 
                break;
          case 2: // L
          case 3: // R
                if (REC_on || MIC_rec_on) { // Is recoding active ?
                  Serial.println("sw: rec stop");
                  pofftm_h = (d_hour * 60 + d_min + 1) / 60;
                  pofftm_m = (d_hour * 60 + d_min + 1) % 60;
                  sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
                  Serial.println(ts); 
                  msg = "control: rec stop scheduled, wait a few minutes";
                  REC_on_no_poff = true;
                } else if (!stop_read){ // playing
                  stop_read = true;
                  p_onoff_req = true;
                  dsp_active = true;
                  pcf.digitalWrite(6, HIGH);
                  audio.stopSong();
                  i2s_stop(I2S_NUM_1);  // stop audio port
                  Serial.println("sw: Play end.");
                } else { // DSP radio mode
                  if (p==2) station_setting_2(); else station_setting();  
                }
                break;
          case 4:  // M
                if (!REC_on && stop_read) {
                  Serial.println("sw: record");
                  total_recorded_size = 0;
                  last_blk = 0;
                  estimated_recorded_size = MAX_RECORD_TIME * SAMPLE_RATE * SAMPLE_BITS * 60 * 2 / 8;  // MAX_RECORD_TIME min
                  pofftm_h = (d_hour * 60 + d_min + MAX_RECORD_TIME) / 60;  // auto stop after MAX_RECORD_TIME min
                  pofftm_m = (d_hour * 60 + d_min + MAX_RECORD_TIME) % 60;
                  sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled");
                  Serial.println(ts); 
                  REC_on_no_poff = true;
                  esp_err_t err = i2s_install("DSP");
                  i2s_setpin("DSP");
                  if (err != ESP_OK) {
                    Serial.println("Failed to initialize I2S!");
                    I2S_err = true;
                  }
                  if(!I2S_err && !SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")){ // SD mount
                    Serial.println("Failed to mount SD Card!");
                    I2S_err = true;
                    i2s_driver_uninstall(I2S_NUM_0);  // no more DSP I2S now
                  }
                  if (!I2S_err) {
                    REC_on = true; // start REC ok
                    REC_on_no_poff = true;
                    //msg = "control: record";
                  } else {
                    //msg = "control: record err";
                  }
                } else {
                  //msg = "control: record ignored";
                }
                break;
          case 5:  // take a photo
                if (camera_ok && !REC_on) {
                  Serial.println("sw: shoot.");
                  shoot_s = true;   
                } else {
                  Serial.println("sw: shoot but camera not initialized or SD busy.");
                }  
                break;
          default: ;
        }
      }
      ct2 = pt2;
    }
  }

  if (stop_read && !v_stream) {  // stop_read is false on ordinary case except SD read mode
    time_t t = time(NULL);
    tm = localtime(&t);
    d_mon  = tm->tm_mon+1;
    d_mday = tm->tm_mday;
    d_hour = tm->tm_hour;
    d_min  = tm->tm_min;
    d_sec  = tm->tm_sec;
    d_wday = tm->tm_wday;
    d_year = tm->tm_year;
    //Serial.print("time ");
    sprintf(wave_filename_t, "/mug%04d%02d%02d%02d%02d%02d_", d_year + 1900, d_mon, d_mday, d_hour, d_min, d_sec);
    sprintf(ts, "%02d-%02d %s", d_mon, d_mday, weekStr[d_wday]);
    if (pt2-pt3 > 100) { // update display time 
      oled.setTextSize(2); // Draw 2X-scale text
      oled.clearDisplay();
      oled.setCursor(0, 0);
      oled.print(ts);
      //Serial.println(ts);
      sprintf(ts,"%02d:%02d:%02d",d_hour,d_min,d_sec);
      oled.setCursor(0, 15);
      oled.print(ts);
      //Serial.println(ts);
      int pi = p_on ? 1 : 0;
      sprintf(ts, "%s%02d %s%01d", "Vol:", vol, "P:", pi);
      oled.setCursor(0, 30);
      oled.print(ts);
      tf = lastfreq/100.0;
      sprintf(ts, "%3.1f S:%03d", tf, radio.getRssi()); // frequency and signal strength
      oled.setCursor(0, 45);
      oled.print(ts);
      oled.display();
    }
    // Camera & SD available, start taking pictures
    if (camera_ok && sd_ok && shoot_s) {
      shoot_s = false; 
      char filename[32];
      sensor_t *s = esp_camera_sensor_get();
      //s->set_vflip(s, 0);
      s->set_framesize(s, FRAMESIZE_XGA);
      //s->set_framesize(s, FRAMESIZE_VGA);
      s->set_exposure_ctrl(s, 1); // on
      s->set_aec2(s, 1);  // auto exposure on 
      s->set_ae_level(s, 2);  // max level 
      s->set_brightness(s, 1);  // max-1 level
      s->set_contrast(s, 1);  // max-1 level
      sprintf(filename, "%s%d.jpg", wave_filename_t, wav_fcount);
      wav_fcount++;
      preferences.putInt("wavf_no", wav_fcount);
      photo_save(filename);
      Serial.printf("Saved picture: %s\r\n", filename);
      photo_Count++;
      preferences.putInt("photo_no", photo_Count);
      //s->set_framesize(s, FRAMESIZE_QQVGA);  // reset
    }

    // check web server req
    server.handleClient();

    if (p_onoff_req) { // power on/off request?
      if (p_on) {
        radio.powerDown();
        Serial.println("pw off");
        p_on = false;
      } else {
        radio.powerUp();
        delay(300);
        radio.setVolume(vol);
        radio.setFrequency(stnFreq[stnIdx]);
        delay(300);
        p_on = true;
      }
      p_onoff_req = false;
    }
    if (lastvol != vol || lastVolume != volume) {
      Serial.println("vol changed");
      if (stop_read) {
        radio.setVolume(vol);
        preferences.putInt("vol_r",vol);
      } else {
        audio.setVolume(volume);
        preferences.putInt("vol",volume);
      }
      vol_ok = true;
      lastvol = vol;
      lastVolume = volume;
    }
    if (laststnIdx != stnIdx) {
      Serial.println("stn changed");
      radio.setFrequency(stnFreq[stnIdx]);
      lastfreq = stnFreq[stnIdx];
      laststnIdx = stnIdx;
      preferences.putInt("stix", stnIdx); // save
      stn_ok = true;
    }
    // check time schedule
    if (last_d_min != d_min) {
      last_d_min = d_min;
      if (pofftm_h == d_hour && pofftm_m == d_min && p_on) { // power off time ?
        p_onoff_req = true;
        pofftm_h = 0;
        pofftm_m = 0;
        if (REC_on || MIC_rec_on) {
          // note : abandon remainning record in the buffer, which is not so important.
          uint8_t wav_header[WAV_HEADER_SIZE];
          file.seek(0);
          if (REC_on) 
            generate_wav_header(wav_header, total_recorded_size, SAMPLE_RATE);
          else
            generate_wav_header(wav_header, total_recorded_size, SAMPLE_RATE_MIC);
          file.write(wav_header, WAV_HEADER_SIZE);
          Serial.printf("WAVE file header updated.\n");
          file.close();
          i2s_driver_uninstall(I2S_NUM_0);
          Serial.printf("Last recorded %d, Total %d bytes.\n", recorded_size, total_recorded_size); 
          Serial.printf("The recording is over.\n");
          recorded_size = 0;
          REC_on = false;
          WAVE_HDR_write = false;
          I2S_err = false;
          if (REC_on_no_poff) {
            p_onoff_req = false;
            REC_on_no_poff = false;
            if (MIC_rec_on) {      
              radio.powerUp();
              dsp_active = true;
            }
          }
          MIC_rec_on = false;
        }
      } else 
      {
        for(int i = 0; i <= MAXSCEDIDX; i++) {
          if (entity[d_wday][i].stime == 0 ) {     
            //nop
            //Serial.println(d_min);
          } else 
          {
            //Serial.println(entity[d_wday][i].stime);
            if ((entity[d_wday][i].stime <= d_hour * 60 + d_min) && 
                ((entity[d_wday][i].stime + entity[d_wday][i].duration) >= (d_hour * 60 + d_min ))
                && (entity[d_wday][i].scheduled != 1)) {
              if (lastfreq == stnFreq[entity[d_wday][i].fidx]) {
                //entity[d_wday][i].scheduled = 1; // mark it scheduled              
              } else {          
                //radio.setFrequency(stnFreq[entity[d_wday][i].fidx]);  #########
                stnIdx =  entity[d_wday][i].fidx;              
                //lastfreq = stnFreq[stnIdx];
              }
              //radio.setVolume(entity[d_wday][i].volstep);
              vol = entity[d_wday][i].volstep;
              currIdx = i;
              entity[d_wday][i].scheduled = 1; // mark it scheduled
              Serial.println("scheduled");
              if (entity[d_wday][i].poweroff==1 || entity[d_wday][i].poweroff==4 || entity[d_wday][i].poweroff==5) { // power off or REC?
                pofftm_h = (entity[d_wday][i].stime + entity[d_wday][i].duration) / 60; // set power off time
                pofftm_m = (entity[d_wday][i].stime + entity[d_wday][i].duration) % 60;
                sprintf(ts,"%02d:%02d %s",pofftm_h,pofftm_m,"poff or recording stop scheduled.");
                Serial.println(ts);
                if (entity[d_wday][i].poweroff==4 || entity[d_wday][i].poweroff==5) 
                  { // REC start
                    total_recorded_size = 0;
                    last_blk = 0;
                    estimated_recorded_size = entity[d_wday][i].duration * SAMPLE_RATE * SAMPLE_BITS * 60 * 2 / 8;  // 
                    esp_err_t err = i2s_install("DSP");
                    i2s_setpin("DSP");
                    if (err != ESP_OK) {
                      Serial.println("Failed to initialize I2S!");
                      I2S_err = true;
                    }
                    if(!I2S_err && !SD.begin(SD_CS, SPI, REC_FREQUENCY, "/sd")){ // SD mount
                      Serial.println("Failed to mount SD Card!");
                      I2S_err = true;
                      i2s_driver_uninstall(I2S_NUM_0); // no more DSP now
                    }
                    if (!I2S_err) {
                      REC_on = true; // start REC ok
                      if (entity[d_wday][i].poweroff==4) REC_on_no_poff = true;
                    }
                  }
                }
                if (p_on==false) {
                  p_onoff_req = true;  //  if power off currently then power on req
                  pofftm_h = 0;        // reset
                  pofftm_m = 0;
                  Serial.println("pw on req");
                }  
              }
          }
        }
      }
    }
    if ((REC_on || MIC_rec_on ) && !I2S_err) {
      // Start recording
      //Serial.println("Start recording");
      uint32_t sample_size = 0;
      uint8_t *rec_buffer = NULL;
      uint32_t avail_size = 0;
      uint32_t BytesWritten = 0;
      avail_size = 1024 * 32;  // 
      if (curr_buf==1) rec_buffer = rec_buffer1; else rec_buffer = rec_buffer2;
      i2s_read(I2S_NUM_0, rec_buffer + recorded_size, avail_size, &sample_size, 1); // read from DSP or MIC
      if (sample_size == 0) {
        //Serial.printf("Record Failed!\n");
        //I2S_err = true;
        ;
      } else { // read ok
        if (MIC_rec_on)
        {// Increase volume
          for (uint32_t i = recorded_size; i < recorded_size + sample_size; i += SAMPLE_BITS/8) {
            (*(uint16_t *)(rec_buffer+i)) <<= 4;
          }
        }
        recorded_size =  recorded_size + sample_size;
        avail_cnt ++;
        if (recorded_size >= K32*2) {
          //Serial.println("Start recording 32k");
          if (curr_buf==1) {
            // switch buffer area
            memcpy(rec_buffer2, rec_buffer1 + (K32*2), recorded_size - (K32*2));
            rec_buffer = rec_buffer2 + recorded_size - (K32*2);
            curr_buf = 2;
          } else { // curr_buff 2
            memcpy(rec_buffer1, rec_buffer2 + (K32*2), recorded_size - (K32*2));
            rec_buffer = rec_buffer1 + recorded_size - (K32*2);
            curr_buf = 1;
          }
          recorded_size = recorded_size - (K32*2);
          SD_write = true; 
        }
      }
      if (SD_write) {
        // write SD
        if (!WAVE_HDR_write) {
          // write wave file header
          sprintf(wave_filename, "%s%d.wav", wave_filename_t, wav_fcount);
          wav_fcount++;
          preferences.putInt("wavf_no", wav_fcount);
          file = SD.open(wave_filename, FILE_WRITE);

          // Write the header to the WAV file
          uint8_t wav_header[WAV_HEADER_SIZE];
          if (REC_on) 
            generate_wav_header(wav_header, estimated_recorded_size, SAMPLE_RATE);
          else 
            generate_wav_header(wav_header, estimated_recorded_size, SAMPLE_RATE_MIC);
          memset(rec_buffer32k, 0, 1024*32);
          //file.write(wav_header, WAV_HEADER_SIZE);
          memcpy(rec_buffer32k, wav_header, WAV_HEADER_SIZE);
          file.write(rec_buffer32k, 1024 * 32); // filler
          total_recorded_size = K32;
          Serial.printf("WAVE file header wrote.\n");
          WAVE_HDR_write = true;
        }
        // write SD data
        if (total_recorded_size/(K32*10) != last_blk) {
           Serial.printf("Available %d times,Left over %d bytes, use buff %d.\n", avail_cnt, recorded_size, curr_buf);
           last_blk = total_recorded_size / (K32*10);
        }
        //Serial.printf("Writing to the file ...\n");
        if (curr_buf==1) rec_buffer = rec_buffer2; else rec_buffer = rec_buffer1;
        int w_sz = file.write(rec_buffer, K32/*recorded_size*/); 
        w_sz = file.write(rec_buffer + K32, K32/*recorded_size*/); 

        //if (file.write(rec_buffer, recorded_size) != recorded_size) {
        if (w_sz != K32/*recorded_size*/) {
          // Retry it, once
          delay(10);
          int w_sz_r = file.write(rec_buffer + w_sz, K32/*recorded_size*/ - w_sz); 
          if (w_sz_r != K32/*recorded_size*/ - w_sz) {
            Serial.printf("Write file and retry Failed! wz:%d, rd:%d\n", w_sz + w_sz_r, recorded_size);
            I2S_err = true;
          } else {
            Serial.printf("Write file failed, and retry success! wz:%d, rd:%d\n", w_sz + w_sz_r, recorded_size);
          }
          total_recorded_size = total_recorded_size + w_sz + w_sz_r;
        } else  total_recorded_size = total_recorded_size + K32*2/*recorded_size*/;
        avail_cnt = 0;       
        SD_write = false;
      }

    }

  } else if (!v_stream)  {  // SD read mode
    if (dsp_active){
      radio.powerDown();
      p_on = false;
      dsp_active = false;
    } 
    if (lastVolume != volume) {
      audio.setVolume(volume);
      preferences.putInt("vol",volume);
      vol_ok = true;
      lastVolume = volume;
    }
    server.handleClient();
    audio.loop();
    if (stop_read) {
      p_onoff_req = true;
      dsp_active = true;
      pcf.digitalWrite(6, HIGH);
      audio.stopSong();
      i2s_stop(I2S_NUM_1); // stop audio port
      Serial.println("Play end.");
    }
  } else { // on v_stream
    server.handleClient();
  }
}
void vol_setting() {
  if (vol_ok) {  // wait last req
     vol_ok = false;
    if (stop_read) { // DSP
      vol++;
      if (vol > 8) vol = 1; // turn around to support single button
    } else { // audio
      volume++;
      if (volume > 12) vol = 1; // turn around to support single button
    }
  }
}
void vol_setting_2() { 
  if (vol_ok) {  // wait last req
    vol_ok = false;
    if (stop_read) { // DSP
      vol--;
      if (vol < 0) {
        vol = 0;
        lastvol = 1;
      } 
    } else { // Audio
      volume--;
      if (volume < 0) {
        volume = 0;
        lastVolume = 1;
      }      
    }
  }
}
void station_setting_2() {
  if (stn_ok) {  // wait last req
    stnIdx--;
    stn_ok = false;
    if (stnIdx < 0) stnIdx = MAXSTNIDX; // turn around to support single button
  }
}
void station_setting() {
  if (stn_ok) {  // wait last req
    stnIdx++;
    stn_ok = false;
    if (stnIdx > MAXSTNIDX) stnIdx = 0;  // turn around to support single button
  }
}
void power_onoff_setting() {
  if (p_onoff_req==false) {  // wait last req
     p_onoff_req = true;  // req
  }
}

int DumpWAVHeader(WavHeader_Struct* Wav) {
  if (memcmp(Wav->RIFFSectionID, "RIFF", 4) != 0) {
    Serial.print("Not a RIFF format file - ");
    PrintData(Wav->RIFFSectionID, 4);
    if (memcmp(Wav->RIFFSectionID, "ID3", 3) == 0) {
      Serial.println(" May be a MP3 format file.");
      return (1);
    }
    return(5);
  } 
  if (memcmp(Wav->RiffFormat, "WAVE", 4) != 0) {
    Serial.print("Not a WAVE file - ");
    PrintData(Wav->RiffFormat, 4);
    return(4);
  }
  if (memcmp(Wav->FormatSectionID, "fmt", 3) != 0) {
    Serial.print("fmt ID not present - ");
    PrintData(Wav->FormatSectionID, 3);
    return(3);
  }
  if (memcmp(Wav->DataSectionID, "data", 4) != 0) {
    Serial.print("data ID not present - ");
    PrintData(Wav->DataSectionID, 4);
    return(2);
  }
  // All looks good, dump the data
  Serial.print("Total size :");
  Serial.println(Wav->Size);
  Serial.print("Format section size :");
  Serial.println(Wav->FormatSize);
  Serial.print("Wave format :");
  Serial.println(Wav->FormatID);
  Serial.print("Channels :");
  Serial.println(Wav->NumChannels);
  Serial.print("Sample Rate :");
  Serial.println(Wav->SampleRate);
  Serial.print("Byte Rate :");
  Serial.println(Wav->ByteRate);
  Serial.print("Block Align :");
  Serial.println(Wav->BlockAlign);
  Serial.print("Bits Per Sample :");
  Serial.println(Wav->BitsPerSample);
  Serial.print("Data Size :");
  Serial.println(Wav->DataSize);
  return(0);
}

void PrintData(const char* Data, uint8_t NumBytes) {
  for (uint8_t i = 0; i < NumBytes; i++)
    Serial.print(Data[i]);
  Serial.println();
}
// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    stop_read = true; // END OF play file
    Serial.print("eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){
    Serial.print("station     ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    if (strlen(info) == 0) stop_read = true; // maybe connect error 
    Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    Serial.print("lasthost    ");Serial.println(info);
}
