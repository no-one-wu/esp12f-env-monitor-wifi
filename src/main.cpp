#include <Arduino.h>  // Arduino 核心库
#include <ESP8266WiFi.h>  // ESP8266 WiFi 库
#include <WiFiUdp.h>  // UDP 通信库
#include <ESP8266HTTPClient.h>  // HTTP 客户端库（用于可能的扩展）

// ========== 配置区 ==========
#define SERIAL_BAUD 115200  // 串口波特率
#define STATUS_LED 2  // GPIO2，低电平点亮  // 状态指示灯引脚
#define HEARTBEAT_INTERVAL 2000UL  // 默认心跳间隔（毫秒）
#define WIFI_RECONNECT_INTERVAL 5000UL  // WiFi 重连间隔（毫秒）
#define DATA_FRAME_START1 0xAA  // 数据帧起始字节1
#define DATA_FRAME_START2 0x55  // 数据帧起始字节2
#define DATA_PAYLOAD_LEN 9  // 数据载荷长度（温度2 + 湿度2 + 光照2 + 火焰1 + 烟雾2）
#define ALARM_FRAME_HEADER 0xEE  // 报警灯帧头
#define ALARM_DEVICE_ID 2  // 报警灯设备ID

struct SensorRecord {  // 传感器数据结构
  float temperature;  // 温度（摄氏度）
  float humidity;  // 湿度（百分比）
  uint16_t illuminance;  // 光照强度
  bool flame;  // 火焰检测（true=检测到）
  uint16_t smoke;  // 烟雾浓度
};

struct CommStats {  // 通信统计结构
  uint32_t totalSend = 0;  // 总发送次数
  uint32_t successSend = 0;  // 成功发送次数
  uint32_t totalReceive = 0;  // 总接收次数
  uint32_t badFrame = 0;  // 坏帧次数
};

struct WiFiConfig {  // WiFi 配置结构
  String ssid = "1";  // WiFi SSID   iQOO Neo9 Pro
  String password = "11111111";  // WiFi 密码
  IPAddress serverIP = IPAddress( 192,168,0,255);  // 服务器IP地址
  uint16_t tcpPort = 4399;  // TCP端口9000
  uint16_t udpPort = 4399;  // UDP端口
  bool useUDP = true;  // 是否使用UDP（false=TCP）
  uint16_t heartbeatMs = HEARTBEAT_INTERVAL;  // 心跳间隔（毫秒）
  bool useStaticIP = true;  // 是否使用静态IP（false=DHCP）
  IPAddress staticIP = IPAddress(192, 168, 0, 56);  // 静态IP地址
  IPAddress gateway = IPAddress(192, 168, 0, 1);  // 网关地址
  IPAddress subnet = IPAddress(255, 255, 255, 0);  // 子网掩码
  IPAddress dns = IPAddress(114, 114, 114, 114);  // DNS服务器（默认114DNS）
};

WiFiClient tcpClient;  // TCP客户端
WiFiUDP udpClient;  // UDP客户端
WiFiConfig conf;  // WiFi配置实例
SensorRecord latestData;  // 最新传感器数据
CommStats stats;  // 通信统计

uint32_t lastHeartbeat = 0;  // 上次心跳时间戳
uint32_t lastWiFiCheck = 0;  // 上次WiFi检查时间戳
uint32_t lastSerialCmd = 0;  // 上次串口命令时间戳

// 串口接收状态机
enum FrameState { WAIT_HEADER1, WAIT_HEADER2, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHECKSUM };  // 帧解析状态枚举
FrameState frameState = WAIT_HEADER1;  // 当前帧状态
uint8_t payloadBuffer[DATA_PAYLOAD_LEN];  // 载荷缓冲区
uint8_t payloadPos = 0;  // 载荷位置
uint8_t frameLen = 0;  // 帧长度

// ========== 工具函数 ==========
void setStatusLED(bool connected) {  // 设置状态LED灯
  digitalWrite(STATUS_LED, connected ? LOW : HIGH);  // 连接时低电平点亮，否则高电平熄灭
}

uint8_t calcChecksum(const uint8_t *buf, uint8_t len) {  // 计算校验和
  uint16_t sum = 0;  // 初始化累加和
  for (uint8_t i = 0; i < len; i++) sum += buf[i];  // 累加每个字节
  return (uint8_t)(sum & 0xFF);  // 返回低8位
}

String sensorDataToJson(const SensorRecord &d) {  // 将传感器数据转换为JSON字符串
  char buf[256];  // JSON缓冲区
  snprintf(buf, sizeof(buf),  // 格式化JSON
    "{\"temperature\":%.2f,\"humidity\":%.2f,\"illuminance\":%u,\"flame\":%s,\"smoke\":%u,\"rssi\":%d,\"ts\":%lu}",
    d.temperature, d.humidity, d.illuminance,
    d.flame ? "true" : "false", d.smoke,
    WiFi.RSSI(), millis());  // 包含温度、湿度、光照、火焰、烟雾、RSSI、时间戳
  return String(buf);  // 返回JSON字符串
}

void printDebugStatus() {  // 打印调试状态信息
  Serial.printf("[INFO] WiFi:%s RSSI:%d dBm, server %s:%u, mode:%s, stat: sent%u/%u recvd%u bad%u\n",
    WiFi.isConnected() ? "OK" : "DISCONN",  // WiFi连接状态
    WiFi.RSSI(),  // 信号强度
    conf.serverIP.toString().c_str(),  // 服务器IP
    conf.useUDP ? conf.udpPort : conf.tcpPort,  // 端口
    conf.useUDP ? "UDP" : "TCP",  // 协议
    stats.successSend, stats.totalSend, stats.totalReceive, stats.badFrame);  // 统计数据
}

// ========== 串口命令（调试配置） ==========
void processSerialCommand(const String &cmd) {  // 处理串口命令
  String line = cmd;  // 复制命令字符串
  line.trim();  // 去除前后空格
  if (line.length() == 0) return;  // 空命令忽略

  if (line.startsWith("SET SSID ")) {  // 设置WiFi SSID
    conf.ssid = line.substring(9);  // 提取SSID
    WiFi.begin(conf.ssid.c_str(), conf.password.c_str());  // 开始连接WiFi
    Serial.println("[CMD] set ssid -> " + conf.ssid);  // 确认输出
    return;
  }
  if (line.startsWith("SET PASS ")) {  // 设置WiFi密码
    conf.password = line.substring(9);  // 提取密码
    WiFi.begin(conf.ssid.c_str(), conf.password.c_str());  // 开始连接WiFi
    Serial.println("[CMD] set password");  // 确认输出
    return;
  }
  if (line.startsWith("SET SERVER ")) {  // 设置服务器地址
    String p = line.substring(11);  // 提取地址部分
    int colon = p.indexOf(':');  // 查找冒号
    if (colon > 0) {  // 如果找到端口
      conf.serverIP.fromString(p.substring(0, colon));  // 设置IP
      conf.tcpPort = p.substring(colon + 1).toInt();  // 设置TCP端口
      conf.udpPort = conf.tcpPort;  // UDP端口同步
      Serial.printf("[CMD] server set %s:%u\n", conf.serverIP.toString().c_str(), conf.tcpPort);  // 确认输出
      return;
    }
  }
  if (line.startsWith("SET MODE ")) {  // 设置通信模式
    String m = line.substring(9);  // 提取模式
    m.toUpperCase();  // 转为大写
    conf.useUDP = (m == "UDP");  // 设置UDP标志
    Serial.println(String("[CMD] mode set ") + (conf.useUDP ? "UDP" : "TCP"));  // 确认输出
    return;
  }
  if (line.startsWith("SET HB ")) {  // 设置心跳间隔
    conf.heartbeatMs = line.substring(7).toInt();  // 提取间隔
    Serial.println("[CMD] heartbeat set " + String(conf.heartbeatMs));  // 确认输出
    return;
  }
  if (line.startsWith("SET STATIC ")) {  // 设置静态IP: SET STATIC <IP>/<网关>/<掩码>/<DNS>
    String p = line.substring(11);  // 提取参数
    int s1 = p.indexOf('/');
    int s2 = p.indexOf('/', s1 + 1);
    int s3 = p.indexOf('/', s2 + 1);
    if (s1 > 0 && s2 > 0 && s3 > 0) {
      conf.staticIP.fromString(p.substring(0, s1));
      conf.gateway.fromString(p.substring(s1 + 1, s2));
      conf.subnet.fromString(p.substring(s2 + 1, s3));
      conf.dns.fromString(p.substring(s3 + 1));
      conf.useStaticIP = true;
      Serial.printf("[CMD] static IP set %s -> reconnect to apply\n", conf.staticIP.toString().c_str());
    }
    return;
  }
  if (line == "SET DHCP") {  // 切换回DHCP模式
    conf.useStaticIP = false;
    Serial.println("[CMD] switched to DHCP -> reconnect to apply");
    return;
  }
  if (line == "STATUS") {  // 查询状态
    printDebugStatus();  // 打印状态
    return;
  }
  Serial.println("[CMD] unknown: " + line);  // 未知命令
}

void readSerialCommands() {  // 读取串口命令
  static String cmdBuf = "";  // 命令缓冲区
  while (Serial.available()) {  // 循环读取可用字符
    char c = Serial.read();  // 读取一个字符
    if (c == '\n' || c == '\r') {  // 如果是换行或回车
      if (cmdBuf.length() > 0) {  // 如果缓冲区有内容
        processSerialCommand(cmdBuf);  // 处理命令
        cmdBuf = "";  // 清空缓冲区
      }
    } else {
      cmdBuf += c;  // 累加字符到缓冲区
    }
  }
}

// ========== 串口数据解析（GD32 输入） ==========
void parseSensorFrame(uint8_t b) {  // 解析传感器数据帧
  switch (frameState) {  // 根据当前状态处理字节
    case WAIT_HEADER1:  // 等待帧头1
      if (b == DATA_FRAME_START1) frameState = WAIT_HEADER2;  // 匹配则进入下一状态
      break;
    case WAIT_HEADER2:  // 等待帧头2
      if (b == DATA_FRAME_START2) frameState = WAIT_LEN;  // 匹配则进入长度状态
      else frameState = WAIT_HEADER1;  // 不匹配重置
      break;
    case WAIT_LEN:  // 等待长度字节
      if (b == DATA_PAYLOAD_LEN) {  // 验证长度
        frameLen = b;  // 设置帧长度
        payloadPos = 0;  // 重置载荷位置
        frameState = WAIT_PAYLOAD;  // 进入载荷状态
      } else {
        frameState = WAIT_HEADER1;  // 长度不匹配重置
      }
      break;
    case WAIT_PAYLOAD:  // 等待载荷数据
      payloadBuffer[payloadPos++] = b;  // 存储字节到缓冲区
      if (payloadPos >= frameLen) frameState = WAIT_CHECKSUM;  // 载荷收集完成进入校验
      break;
    case WAIT_CHECKSUM: {  // 等待校验字节
      uint8_t ck = calcChecksum(payloadBuffer, frameLen);  // 计算校验和
      if (ck == b) {  // 校验通过
        // 解包数据
        int16_t t100 = (int16_t)((payloadBuffer[0] << 8) | payloadBuffer[1]);  // 温度（x100）
        int16_t h100 = (int16_t)((payloadBuffer[2] << 8) | payloadBuffer[3]);  // 湿度（x100）
        latestData.temperature = t100 / 100.0f;  // 转换为实际温度
        latestData.humidity = h100 / 100.0f;  // 转换为实际湿度
        latestData.illuminance = (uint16_t)((payloadBuffer[4] << 8) | payloadBuffer[5]);  // 光照
        latestData.flame = payloadBuffer[6] != 0;  // 火焰检测
        latestData.smoke = (uint16_t)((payloadBuffer[7] << 8) | payloadBuffer[8]);  // 烟雾
        stats.totalReceive++;  // 接收计数增加
        Serial.printf("[DATA] T=%.2f H=%.2f Lux=%u flame=%d smoke=%u\n",  // 打印解析数据
          latestData.temperature, latestData.humidity,
          latestData.illuminance, latestData.flame ? 1 : 0,
          latestData.smoke);
      } else {
        stats.badFrame++;  // 坏帧计数增加
        Serial.println("[WARN] frame checksum mismatch");  // 校验失败警告
      }
      frameState = WAIT_HEADER1;  // 重置状态机
      break;
    }
  }
}

void readSensorSerial() {  // 读取传感器串口数据
  while (Serial.available()) {  // 循环读取可用字节
    uint8_t b = Serial.read();  // 读取一个字节
    parseSensorFrame(b);  // 交给状态机解析
  }
}

void sendAlarmToMCU() {  // 向MCU发送报警灯触发帧: 0xEE 0x01 0xFE
  uint8_t frame[3] = { ALARM_FRAME_HEADER, 0x01, (uint8_t)(0x01 ^ 0xFF) };  // 帧头 + CMD + ~CMD
  Serial.write(frame, 3);  // 通过串口发给MCU
  Serial.println("[ALARM] sent to MCU: EE 01 FE");  // 日志
}

// ========== TCP/UDP 传输 ==========
bool ensureTcpConnected() {  // 确保TCP连接
  if (tcpClient.connected()) return true;  // 已连接直接返回
  if (tcpClient.connect(conf.serverIP, conf.tcpPort)) {  // 尝试连接
    Serial.println("[NET] TCP connected");  // 连接成功日志
    return true;
  }
  return false;  // 连接失败
}

bool sendDataToServer(const String &json) {  // 发送数据到服务器
  bool ok = false;  // 发送结果标志
  if (conf.useUDP) {  // 如果使用UDP
    udpClient.beginPacket(conf.serverIP, conf.udpPort);  // 开始UDP包
    udpClient.print(json);  // 发送数据
    ok = udpClient.endPacket() == 1;  // 检查发送结果
  } else {  // 如果使用TCP
    if (ensureTcpConnected()) {  // 确保TCP连接
      unsigned int n = tcpClient.print(json);  // 发送数据，返回发送字节数
      ok = (n == json.length());  // 检查是否全部发送
      if (!ok) {
        tcpClient.stop();  // 发送失败断开连接
      }
    } else {
      ok = false;  // 连接失败
    }
  }

  stats.totalSend++;  // 总发送计数增加
  if (ok) stats.successSend++;  // 成功发送计数增加
  return ok;  // 返回发送结果
}

void periodicSend() {  // 周期性发送数据
  if (WiFi.isConnected() && stats.totalReceive > 0) {  // WiFi连接且有数据
    String payload = sensorDataToJson(latestData);  // 生成JSON数据
    if (sendDataToServer(payload)) {  // 发送数据
      Serial.println("[SEND] ok " + payload);  // 发送成功日志
    } else {
      Serial.println("[SEND] fail " + payload);  // 发送失败日志
    }
  }
}

// ========== WiFi 连接管理 ==========
void updateBroadcastIP() {  // 根据当前网络自动计算正确的广播地址
  IPAddress local = WiFi.localIP();  // 获取本机IP
  IPAddress mask = WiFi.subnetMask();  // 获取子网掩码
  conf.serverIP = IPAddress(
    (uint32_t)local | ~(uint32_t)mask  // 计算广播地址: localIP | ~subnetMask
  );
  Serial.printf("[WIFI] auto broadcast -> %s\n", conf.serverIP.toString().c_str());  // 打印广播地址
  conf.udpPort = conf.tcpPort;  // 同步端口
}

void connectWiFi() {  // 连接WiFi
  if (WiFi.isConnected()) return;  // 已连接则返回
  Serial.printf("[WIFI] connecting %s ...\n", conf.ssid.c_str());  // 连接日志
  WiFi.mode(WIFI_STA);  // 设置为STA模式设置为客户端
  if (conf.useStaticIP) {  // 如果启用静态IP
    WiFi.config(conf.staticIP, conf.gateway, conf.subnet, conf.dns);  // 设置静态IP
    Serial.printf("[WIFI] static IP: %s\n", conf.staticIP.toString().c_str());  // 打印静态IP
  }
  WiFi.begin(conf.ssid.c_str(), conf.password.c_str());  // 开始连接
}

void checkWiFi() {  // 检查WiFi连接状态
  if (millis() - lastWiFiCheck < WIFI_RECONNECT_INTERVAL) return;  // 未到检查时间返回
  lastWiFiCheck = millis();  // 更新检查时间

  static bool wasDisconnected = true;  // 跟踪连接状态变化
  if (!WiFi.isConnected()) {  // 如果未连接
    connectWiFi();  // 尝试连接
    wasDisconnected = true;  // 标记为断开
    setStatusLED(false);  // 熄灭LED
  } else {
    setStatusLED(true);  // 点亮LED
    if (wasDisconnected) {  // 刚连上
      updateBroadcastIP();  // 自动计算广播地址
      wasDisconnected = false;  // 标记为已连接
    }
  }
}

// ========== 软看门狗与异常处理 ==========
void handleWatchdog() {  // 处理看门狗
  ESP.wdtFeed();  // 喂狗防止重启
}

// ========== 代码起始位置 ==========
void setup() {  // 初始化函数
  pinMode(STATUS_LED, OUTPUT);  // 设置LED引脚为输出
  setStatusLED(false);  // 初始化LED熄灭

  Serial.begin(SERIAL_BAUD);  // 初始化串口
  while (!Serial) ;  // 等待串口就绪
  Serial.setTimeout(10);  // 设置串口超时

  Serial.println("[BOOT] ESP-12F 双向数据系统启动");  // 启动日志

  connectWiFi();  // 连接WiFi

  if (!conf.useUDP) {  // 如果使用TCP
    tcpClient.setNoDelay(true);  // 设置TCP无延迟
  }

  lastHeartbeat = millis();  // 初始化心跳时间戳记录当前时间段
  lastWiFiCheck = millis();  // 初始化WiFi检查时间戳
}

void loop() {  // 主循环
  handleWatchdog();  // 喂狗

  // 区分命令和传感器数据：如果第一个字节是帧头，则处理传感器数据，否则处理命令
  if (Serial.available() > 0) {
    uint8_t firstByte = Serial.peek();  // 查看第一个字节，不消耗
    if (firstByte == DATA_FRAME_START1) {  // 如果是帧头，处理传感器数据
      readSensorSerial();
    } else {  // 否则处理串口命令
      readSerialCommands();
    }
  }

  checkWiFi();  // 检查WiFi状态

  if (conf.useUDP) {  // 如果使用UDP
    // UDP无需保持连接
  } else {  // 如果使用TCP
    if (tcpClient.connected() && tcpClient.available()) {  // TCP连接且有数据
      String down = tcpClient.readStringUntil('\n');  // 读取下行数据
      if (down.length() > 0) {  // 如果有数据
        Serial.println("[DOWN] " + down);  // 打印下行数据
        if (down.indexOf("\"t\":\"alarm\"") >= 0) {  // 服务器下发报警指令
          sendAlarmToMCU();  // 转发报警帧给MCU
        } else {  // 否则作为配置命令处理
          processSerialCommand(down);  // 处理下行命令
        }
      }
    }
  }

  if (millis() - lastHeartbeat >= conf.heartbeatMs) {  // 心跳时间到
    lastHeartbeat = millis();  // 更新心跳时间戳
    periodicSend();  // 发送数据
    printDebugStatus();  // 打印状态
  }

  // 允许触发串口调试状态
  if (millis() - lastSerialCmd > 2000) {  // 串口命令超时
    if (Serial.available()) {  // 如果串口有数据
      lastSerialCmd = millis();  // 更新串口命令时间戳
    }
  }

  delay(10);  // 延时10ms
}
