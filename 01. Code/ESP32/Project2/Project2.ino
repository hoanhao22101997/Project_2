#include <WiFi.h>
#include <ESP32Firebase.h>
#include <HTTPClient.h>
#include  <Adafruit_ST7735.h>
#include  <Adafruit_GFX.h>
#include  <SPI.h>
const unsigned char uteute_min [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x2c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x00, 0x00, 0x00, 0x00, 0x01, 0xc0, 0x34, 0x03, 
	0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x01, 0x80, 0x00, 0x00, 0x03, 0x80, 0x00, 0x01, 0xc0, 0x00, 
	0x00, 0x03, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x07, 0x00, 0x18, 0x00, 0xe0, 0x00, 0x00, 0x06, 
	0x00, 0x5c, 0x00, 0x60, 0x00, 0x00, 0x06, 0x00, 0x5a, 0x00, 0x60, 0x00, 0x00, 0x0e, 0x60, 0xda, 
	0x06, 0x70, 0x00, 0x00, 0x0d, 0xb0, 0x52, 0x0d, 0xb0, 0x00, 0x00, 0x0c, 0xd0, 0x7c, 0x0b, 0x30, 
	0x00, 0x00, 0x0c, 0xe2, 0x18, 0x47, 0x30, 0x00, 0x00, 0x0c, 0xf0, 0x10, 0x5f, 0x30, 0x00, 0x00, 
	0x0c, 0xf9, 0x00, 0x9f, 0x70, 0x00, 0x00, 0x0e, 0xfe, 0x00, 0x7f, 0x60, 0x00, 0x00, 0x06, 0x7f, 
	0x81, 0xfe, 0x60, 0x00, 0x00, 0x06, 0x0f, 0x81, 0xf0, 0x60, 0x00, 0x00, 0x07, 0x07, 0x81, 0xe0, 
	0xe0, 0x00, 0x00, 0x03, 0x0f, 0x81, 0xf0, 0xc0, 0x00, 0x00, 0x03, 0x8f, 0x81, 0xf1, 0x80, 0x00, 
	0x00, 0x01, 0xcf, 0x81, 0xf3, 0x80, 0x00, 0x00, 0x00, 0xef, 0x81, 0xf7, 0x00, 0x00, 0x00, 0x00, 
	0x7f, 0x81, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x81, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x81, 
	0xf8, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1b, 0x39, 
	0xba, 0x7f, 0x78, 0x00, 0x00, 0x1b, 0x6d, 0xba, 0x4c, 0x40, 0x00, 0x00, 0x1b, 0x61, 0xba, 0x4c, 
	0x40, 0x00, 0x00, 0x1f, 0x61, 0xfa, 0x4c, 0x70, 0x00, 0x00, 0x1b, 0x6d, 0x5a, 0x4c, 0x40, 0x00, 
	0x00, 0x1b, 0x79, 0x5b, 0xcc, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
const unsigned char feeefeee [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x80, 0x00, 0x00, 0x03, 
	0x8c, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x80, 0x40, 0x01, 0x80, 0x38, 0x60, 
	0x01, 0x00, 0x00, 0x03, 0xe0, 0x03, 0xe0, 0x01, 0x00, 0x20, 0x03, 0xf8, 0x0f, 0xe0, 0x00, 0x00, 
	0x20, 0x00, 0xff, 0x3f, 0x80, 0x02, 0x00, 0x00, 0x00, 0x3f, 0xfe, 0x00, 0x02, 0x00, 0x10, 0x00, 
	0x0f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x03, 0x83, 0xe0, 0xe0, 0x04, 0x00, 0x08, 0x00, 0xc3, 0xe1, 
	0x80, 0x04, 0x00, 0x08, 0x00, 0x03, 0xe0, 0x00, 0x00, 0x00, 0x04, 0x03, 0x83, 0xe0, 0xe0, 0x08, 
	0x00, 0x04, 0x01, 0xc3, 0xe1, 0xc0, 0x08, 0x00, 0x04, 0x00, 0x43, 0xe1, 0x00, 0x10, 0x00, 0x02, 
	0x00, 0x03, 0xe0, 0x00, 0x10, 0x00, 0x02, 0x00, 0x03, 0xe0, 0x00, 0x10, 0x00, 0x01, 0x00, 0x03, 
	0xe0, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x01, 0x00, 0x07, 0xfc, 0x00, 
	0x20, 0x00, 0x01, 0x83, 0xff, 0xff, 0xf8, 0x40, 0x00, 0x0f, 0xbe, 0x77, 0x38, 0x3f, 0x7c, 0x00, 
	0x3f, 0xfe, 0x77, 0x3b, 0xff, 0xff, 0x00, 0x7f, 0xff, 0x77, 0x32, 0x7f, 0xff, 0x80, 0x3f, 0xff, 
	0x67, 0x33, 0xff, 0xff, 0x00, 0x0f, 0xff, 0x9f, 0xfe, 0x7f, 0xf8, 0x00, 0x00, 0x7f, 0xff, 0xff, 
	0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x7e, 0x7e, 0x7e, 0x7e, 0x00, 0x00, 0x00, 0x70, 0x60, 0x70, 0x70, 0x00, 0x00, 0x00, 0x74, 0x64, 
	0x74, 0x34, 0x00, 0x00, 0x00, 0x7c, 0x7c, 0x7c, 0x3c, 0x00, 0x00, 0x00, 0x70, 0x70, 0x70, 0x70, 
	0x00, 0x00, 0x00, 0x70, 0x73, 0x73, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
constexpr int Pin_LCD_CS = 27;
constexpr int Pin_LCD_RS = 23;
constexpr int Pin_LCD_RST = 22;
constexpr int Pin_LCD_SCLK = 14;
constexpr int Pin_LCD_MISO = 12;
constexpr int Pin_LCD_SDA = 13;
Adafruit_ST7735 tft = Adafruit_ST7735(Pin_LCD_CS, Pin_LCD_RS, Pin_LCD_SDA, Pin_LCD_SCLK, Pin_LCD_RST);  // CS, DC, MOSI, SCLK, RST


const char* ssid = "PQ";              // Tên mạng WiFi của bạn
const char* password = "99999999";    // Mật khẩu mạng của bạn

#define REFERENCE_URL "https://project2-ec524-default-rtdb.firebaseio.com/"  // URL tham chiếu Firebase của bạn

WiFiClient client;
Firebase firebase(REFERENCE_URL);

uint8_t id, rw, temp,humi,crc;
uint16_t ppm,co;
#define RXD2 16  // Chân RX cho UART2 (nhận dữ liệu)
#define TXD2 17  // Chân TX cho UART2 (gửi dữ liệu)
#define ID_ESP 0x03
#define ID_STM32_1 0x01
#define ID_STM32_2 0x02
int tick =0;
typedef struct {
	uint8_t id;        // ID 8 bit
	uint8_t rw;        // Read (0) hoặc Write (1) 1 bit
	uint8_t d_humi;
	uint8_t d_temp;
	uint16_t d_ppm;
	uint16_t d_co;
  uint8_t speed;
} LoRaFrame;
LoRaFrame LoRaRx1;
LoRaFrame LoRaRx2;

LoRaFrame LoRaTx;;
// uint8_t CalculateCRC(LoRaFrame *frame, uint16_t dataLength) {
// 	uint8_t crc = 0;
// 	crc ^= frame->id;
// 	crc ^= frame->rw;

// 	// Tính CRC cho từng byte trong chuỗi dữ liệu
// 	for (uint16_t i = 0; i < dataLength; i++) {
// 		crc ^= frame->data[i];
// 	}

// 	return crc;
// }

void LoRa_SendFrame(uint8_t id, uint8_t rw, LoRaFrame *data) {
//	LoRaFrame frame;
//	frame.id = data->id;            // ID 8-bit
//	frame.rw = data->rw & 0x01;     // �?ảm bảo RW là 1 bit (0 hoặc 1)
//	uint8_t end = sizeof(frame.data) - 1;
//	frame.data[end] = '/0';
//	uint8_t len = strlen(data);
//	frame.crc = CalculateCRC(&frame, len);
	char buf[258];
	sprintf(buf, "ID:%u RW:%u HUMI:%u TEMP:%u PPM:%Lu CO:%Lu \n\r", data->id, data->rw,
			data->d_humi, data->d_temp,data->d_ppm,data->d_co);
	// Gửi khung dữ liệu qua UART
  Serial2.write(buf);
	// HAL_UART_Transmit(&huart1, (uint8_t*) buf, strlen(buf), 100);
}
void UpdateTFT(LoRaFrame *pLoraFrame1, LoRaFrame *pLoraFrame2) {
  char buf_t1[50];
  char buf_t2[50];

  char buf_h1[50];
  char buf_h2[50];

  char buf_co1[50];
  char buf_co2[50];

  char buf_ppm1[50];
  char buf_ppm2[50];

  char buf_sp1[50];
  char buf_sp2[50];

  sprintf(buf_t1, "Temp:%u C", pLoraFrame1->d_temp);
  sprintf(buf_t2, "Temp:%u C", pLoraFrame2->d_temp);

  sprintf(buf_h1, "Humi:%u%%", pLoraFrame1->d_humi);
  sprintf(buf_h2, "Humi:%u%%", pLoraFrame2->d_humi);

  sprintf(buf_co1, "CO:%Lu%%", pLoraFrame1->d_co);
  sprintf(buf_co2, "CO:%Lu%%", pLoraFrame2->d_co);

  sprintf(buf_ppm1, "PPM:%Lu%%", pLoraFrame1->d_ppm);
  sprintf(buf_ppm2, "PPM:%Lu%%", pLoraFrame2->d_ppm);

  sprintf(buf_sp1, "Speed:%u%%", pLoraFrame1->speed);
  sprintf(buf_sp2, "Speed:%u%%", pLoraFrame2->speed);

  tft.setRotation(1);
  tft.setTextSize(1);

  // Xóa từng vùng trước khi cập nhật
  tft.fillRect(40, 60, 80, 10, ST7735_BLACK);   // Xóa vùng NODE GATEWAY
  tft.fillRect(0, 70, 40, 10, ST7735_BLACK);    // Xóa vùng NODE1
  tft.fillRect(90, 70, 40, 10, ST7735_BLACK);   // Xóa vùng NODE2

  tft.fillRect(0, 80, 80, 10, ST7735_BLACK);    // Xóa vùng Temp NODE1
  tft.fillRect(90, 80, 80, 10, ST7735_BLACK);   // Xóa vùng Temp NODE2

  tft.fillRect(0, 90, 80, 10, ST7735_BLACK);    // Xóa vùng Humi NODE1
  tft.fillRect(90, 90, 80, 10, ST7735_BLACK);   // Xóa vùng Humi NODE2

  tft.fillRect(0, 100, 80, 10, ST7735_BLACK);   // Xóa vùng CO NODE1
  tft.fillRect(90, 100, 80, 10, ST7735_BLACK);  // Xóa vùng CO NODE2

  tft.fillRect(0, 110, 80, 10, ST7735_BLACK);   // Xóa vùng PPM NODE1
  tft.fillRect(90, 110, 80, 10, ST7735_BLACK);  // Xóa vùng PPM NODE2

  tft.fillRect(0, 120, 80, 10, ST7735_BLACK);   // Xóa vùng Speed NODE1
  tft.fillRect(90, 120, 80, 10, ST7735_BLACK);  // Xóa vùng Speed NODE2

  // Hiển thị nội dung mới
  tft.setTextColor(ST7735_RED);
  tft.setCursor(40, 60);
  tft.print("NODE GATEWAY");

  tft.setCursor(0, 70);
  tft.print("NODE1");
  tft.setCursor(90, 70);
  tft.print("NODE2");

  tft.setTextColor(ST7735_BLUE);
  tft.setCursor(0, 80);
  tft.print(buf_t1);
  tft.setCursor(90, 80);
  tft.print(buf_t2);

  tft.setTextColor(ST7735_BLUE);
  tft.setCursor(0, 90);
  tft.print(buf_h1);
  tft.setCursor(90, 90);
  tft.print(buf_h2);

  tft.setTextColor(ST7735_YELLOW);
  tft.setCursor(0, 100);
  tft.print(buf_co1);
  tft.setCursor(90, 100);
  tft.print(buf_co2);

  tft.setTextColor(ST7735_GREEN);
  tft.setCursor(0, 110);
  tft.print(buf_ppm1);
  tft.setCursor(90, 110);
  tft.print(buf_ppm2);

  tft.setTextColor(ST7735_GREEN);
  tft.setCursor(0, 120);
  tft.print(buf_sp1);
  tft.setCursor(90, 120);
  tft.print(buf_sp2);
}


void setup() {
  // Khởi tạo UART0 với baud rate 115200 (giao tiếp với máy tính qua Serial Monitor)
  Serial.begin(115200);
  
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.fillScreen(ST7735_BLACK);
  tft.enableDisplay(true);
  UpdateTFT(&LoRaRx1,&LoRaRx2);

  tft.drawBitmap(0, 0, uteute_min, 56, 56, ST7735_WHITE);
  tft.drawBitmap(60, 0, feeefeee, 50, 50, ST7735_WHITE);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(1000);

  Serial.println();
  Serial.println();
  Serial.print("Đang kết nối tới: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("-");
  }
  Serial.println("");
  Serial.println("Đã kết nối WiFi");
  Serial.print("Địa chỉ IP: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
  Serial.println("/");
  // Khởi tạo UART2 với baud rate 9600 (dùng cho giao tiếp với thiết bị khác)
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // SERIAL_8N1: 8 bit dữ liệu, không chẵn lẻ, 1 bit stop

  Serial.println("UART2 initialized on GPIO 16 (RX) and GPIO 17 (TX)");
  tick = millis();
}

void loop() {
  // Kiểm tra nếu có dữ liệu từ UART2 (thiết bị kết nối)
if (Serial2.available()) {
    String a;      // Khai báo biến kiểu String
    String b;      // Sử dụng String cho dữ liệu
    a = Serial2.readStringUntil('\r');  // Đọc dữ liệu từ Serial2 cho đến khi gặp '\r'

    // Chuyển đổi String thành mảng ký tự để sử dụng trong sscanf
    char buffer[255];
    a.toCharArray(buffer, sizeof(buffer));  // Chuyển đổi thành mảng ký tự

    // Cắt dữ liệu từ chuỗi
    sscanf(buffer, "ID:%hhu RW:%hhu HUMI:%hhu TEMP:%hhu PPM:%hu CO:%hu \n\r", &LoRaRx1.id, &LoRaRx1.rw, &LoRaRx1.d_humi,&LoRaRx1.d_temp,&LoRaRx1.d_ppm,&LoRaRx1.d_co);

    // In ra kết quả
    // In ra kết quả
        Serial.print("Received: ");
        Serial.println(buffer);  // In ra chuỗi đã nhận
    if(LoRaRx1.id == ID_STM32_1){
      Serial.print("Dung ID STM32 1");
      if(LoRaRx1.rw == 1){
        Serial.print("STM32 gui toi ESP voi quyen la gui");
        Serial.print("Received: ");
        Serial.println(buffer);  // In ra chuỗi đã nhận
        Serial.print("ID: ");
        Serial.println(LoRaRx1.id);      // In ra ID
        Serial.print("RW: ");
        Serial.println(LoRaRx1.rw);      // In ra RW
        Serial.print("HUMI: ");
        Serial.println(LoRaRx1.d_humi);    // In ra độ ẩm
        Serial.print("TEMP: ");
        Serial.println(LoRaRx1.d_temp);    // In ra nhiệt độ
        Serial.print("PPM: ");
        Serial.println(LoRaRx1.d_ppm);     // In ra PPM
        Serial.print("CO: ");
        Serial.println(LoRaRx1.d_co);      // In ra CO
        Serial.print("\n");
        firebase.setInt("/STM32_1/CO", LoRaRx1.d_co);
        firebase.setInt("/STM32_1/HUMI", LoRaRx1.d_humi);
        firebase.setInt("/STM32_1/PPM", LoRaRx1.d_ppm);
        firebase.setInt("/STM32_1/TEMP", LoRaRx1.d_temp);
        Serial.println("Da Gui Len FireBase xong");
      }else{
        Serial.print("STM32 gui toi ESP voi quyen la nhan");
        //TODO
      }
    }
    UpdateTFT(&LoRaRx1, &LoRaRx2);

    // LoRaTx.id = id;
    // LoRaTx.rw = rw;
    // LoRaTx.d_temp = temp;
    // LoRaTx.d_humi = humi,
    // LoRaTx.d_ppm = ppm;
    // LoRaTx.d_co = co;
    // LoRa_SendFrame(0x04,1,&LoRaTx);
}
  if(millis() - tick >= 1000){
    // LoRa_SendFrame(ID_ESP,1,&LoRaTx);
    Serial2.write(ID_ESP);
    Serial.println("DA GUI XUONG STM32 1");
    tick = millis();
  }
  if(millis() - tick >= 1200){
    // LoRa_SendFrame(ID_ESP,1,&LoRaTx);
    Serial2.write(ID_ESP);
    Serial.println("DA GUI XUONG STM32 1");
    tick = millis();
  }
  // Serial2.println("Hello World");
  // delay(500);

  // // Gửi dữ liệu từ Serial Monitor tới UART2
  // if (Serial.available()) {
  //   String input = Serial.readString();  // Đọc dữ liệu từ máy tính
  //   Serial2.println(input);              // Gửi dữ liệu tới UART2
  //   Serial.println("Sent to UART2: " + input);  // Xác nhận đã gửi
  // }
}
