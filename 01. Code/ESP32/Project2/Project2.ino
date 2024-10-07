#include <WiFi.h>
#include <ESP32Firebase.h>
#include <HTTPClient.h>

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

} LoRaFrame;
LoRaFrame LoRaRx;
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

void setup() {
  // Khởi tạo UART0 với baud rate 115200 (giao tiếp với máy tính qua Serial Monitor)
  Serial.begin(115200);
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
    sscanf(buffer, "ID:%hhu RW:%hhu HUMI:%hhu TEMP:%hhu PPM:%hu CO:%hu \n\r", &id, &rw, &humi,&temp,&ppm,&co,&crc);

    // In ra kết quả
    // In ra kết quả
        Serial.print("Received: ");
        Serial.println(buffer);  // In ra chuỗi đã nhận
    if(id == ID_STM32_1){
      Serial.print("Dung ID STM32 1");
      if(rw == 1){
        Serial.print("STM32 gui toi ESP voi quyen la gui");
        Serial.print("Received: ");
        Serial.println(buffer);  // In ra chuỗi đã nhận
        Serial.print("ID: ");
        Serial.println(id);      // In ra ID
        Serial.print("RW: ");
        Serial.println(rw);      // In ra RW
        Serial.print("HUMI: ");
        Serial.println(humi);    // In ra độ ẩm
        Serial.print("TEMP: ");
        Serial.println(temp);    // In ra nhiệt độ
        Serial.print("PPM: ");
        Serial.println(ppm);     // In ra PPM
        Serial.print("CO: ");
        Serial.println(co);      // In ra CO
        Serial.print("\n");
        firebase.setInt("/STM32_1/CO", co);
        firebase.setInt("/STM32_1/HUMI", humi);
        firebase.setInt("/STM32_1/PPM", ppm);
        firebase.setInt("/STM32_1/TEMP", temp);
        Serial.println("Da Gui Len FireBase xong");
      }else{
        Serial.print("STM32 gui toi ESP voi quyen la nhan");
        //TODO
      }
    }
    // LoRaTx.id = id;
    // LoRaTx.rw = rw;
    // LoRaTx.d_temp = temp;
    // LoRaTx.d_humi = humi,
    // LoRaTx.d_ppm = ppm;
    // LoRaTx.d_co = co;
    // LoRa_SendFrame(0x04,1,&LoRaTx);
}
  if(millis() - tick >= 5000){
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
