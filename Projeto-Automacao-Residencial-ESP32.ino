/******************************************************************************************************************************************
  IoT - Automação Residencial
  Autor : Robson Brasil

  Dispositivos : ESP32 WROOM32, DHT22, BMP180, Módulo Relé de 8 Canais
  Preferences--> URLs adicionais do Gerenciador de placas:
                                    ESP8266: http://arduino.esp8266.com/stable/package_esp8266com_index.json,
                                    ESP32  : https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  Download Board ESP32 (x.x.x):
  Broker MQTT, Retain, QoS, KeepAlive
  OTA -Over-the-Air
  Node-Red / Google Assistant-Nora:  https://smart-nora.eu/
  Para Instalação do Node-Red:       https://nodered.org/docs/getting-started/
  Home Assistant
  Para Instalação do Home Assistant: https://www.home-assistant.io/installation/
  Versão : 1.0 - Release Candidate
  Última Modificação : 20/06/2025
******************************************************************************************************************************************/

// =============== IP FIXO ===============
#include <DNSServer.h>
DNSServer dns;
IPAddress local_IP(192, 168, 15, 160);  //<-- Coloque aqui o Ip referente a tua rede
IPAddress gateway(192, 168, 15, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(1, 1, 1, 1);
IPAddress secondaryDNS(8, 8, 8, 8);

// =============== DEFINIÇÕES E CONSTANTES ===============
// Definições dos pinos dos Relés
#define RelayPin1 26  // D26 Ligados ao Nora/MQTT
#define RelayPin2 13  // D13 Ligados ao Nora/MQTT
#define RelayPin3 14  // D14 Ligados ao Nora/MQTT
#define RelayPin4 32  // D32 Ligados ao Nora/MQTT
#define RelayPin5 16  // D16 Ligados ao Nora/MQTT
#define RelayPin6 17  // D17 Ligados ao MQTT
#define RelayPin7 18  // D18 Ligados ao MQTT
#define RelayPin8 19  // D19 Ligados ao MQTT

// =============== BIBLIOTECAS ===============
#include "DefinicoesProjeto.h"
#include "Bibliotecas.h"
#include "GPIOs.h"
#include "VariaveisGlobais.h"
#include "LoginsSenhas.h"
#include "ConfigMQTT.h"
#include "TopicosMQTT.h"
#include "TaskCores.h"
#include "ConstantesTempo.h"
#include "Sensores.h"
#include "MQTTParametros.h"
#include "WiFiUtils.h"

// LED indicador de WiFi
#define wifiLed    2  // D2

// Sensor DHT
#define DHTPIN    25  // D25
#define DHTTYPE   DHT22  // DHT22 (AM2302) - Altere para DHT11 se estiver usando esse modelo

// Pinos I2C para BMP180
#define I2C_SDA   21  // Pino SDA
#define I2C_SCL   22  // Pino SCL

// Estados dos relés
bool RelayState1 = false;
bool RelayState2 = false;
bool RelayState3 = false;
bool RelayState4 = false;
bool RelayState5 = false;
bool RelayState6 = false;
bool RelayState7 = false;
bool RelayState8 = false;

// =============== INSTÂNCIAS DE OBJETOS ===============
// Instância do sensor DHT
DHT dht(DHTPIN, DHTTYPE);

// Instância do sensor BMP180
Adafruit_BMP085 bmp;
WiFiClient espClient;          // Cliente WiFi
PubSubClient mqttClient(espClient); // Cliente MQTT

// =============== VARIÁVEIS GLOBAIS ===============
// Configurações do WiFi
const char* ssid     = "xxxxxxxx";  // SSID da rede WiFi, não se esqueça de preencher  corretamente aqui
const char* password = "xxxxxxxx";  // Senha da rede WiFi, não esquecça de preencher corretamente aqui

// Variáveis para armazenar os dados dos sensores
char str_hum_data[7];        // Umidade
char str_temp_data[7];       // Temperatura (Celsius)
char str_tempterm_data[7];   // Temperatura térmica
char str_tempF_data[7];      // Temperatura (Fahrenheit)  // Variáveis para armazenar os dados do BMP180
float temperature = 0;
float pressure = 0;
float altitude = 0;
float altitudeTotal = 0;

// Variáveis para controle de tempo de leitura dos sensores
// =============== VARIÁVEIS DE CONTROLE DE TEMPO ===============
unsigned long lastWifiRetryTime = 0;
unsigned long lastWifiBlinkTime = 0;
unsigned long lastWifiCheckTime = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastRelayUpdate = 0;

// =============== CONSTANTES DE TEMPO ===============
const unsigned long WIFI_RETRY_INTERVAL = 1000;
const unsigned long WIFI_BLINK_INTERVAL = 500;
const unsigned long RECONNECT_INTERVAL = 5000;
const unsigned long DHT_READ_INTERVAL = 60000;
const unsigned long BMP_READ_INTERVAL = 120000;

// =============== VARIÁVEIS DE ESTADO ===============
bool wifiConnected = false;
bool mqttConnected = false;

// =============== TÓPICOS MQTT ===============
// Tópicos Subscribe (recebimento)
const char* sub0 = "ESP32/MinhaCasa/QuartoRobson/Ligar-DesligarTudo/Estado"; 	// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* sub1 = "ESP32/MinhaCasa/QuartoRobson/Interruptor1/Estado";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* sub2 = "ESP32/MinhaCasa/QuartoRobson/Interruptor2/Estado";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* sub3 = "ESP32/MinhaCasa/QuartoRobson/Interruptor3/Estado";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* sub4 = "ESP32/MinhaCasa/QuartoRobson/Interruptor4/Estado";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* sub5 = "ESP32/MinhaCasa/QuartoRobson/Interruptor5/Estado";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* sub6 = "ESP32/MinhaCasa/QuartoRobson/Interruptor6/Estado";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* sub7 = "ESP32/MinhaCasa/QuartoRobson/Interruptor7/Estado";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* sub8 = "ESP32/MinhaCasa/QuartoRobson/Interruptor8/Estado";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
// Tópicos Publish (envio)
const char* pub1 = "ESP32/MinhaCasa/QuartoRobson/Interruptor1/Comando";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub2 = "ESP32/MinhaCasa/QuartoRobson/Interruptor2/Comando";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub3 = "ESP32/MinhaCasa/QuartoRobson/Interruptor3/Comando";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub4 = "ESP32/MinhaCasa/QuartoRobson/Interruptor4/Comando";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub5 = "ESP32/MinhaCasa/QuartoRobson/Interruptor5/Comando";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub6 = "ESP32/MinhaCasa/QuartoRobson/Interruptor6/Comando";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub7 = "ESP32/MinhaCasa/QuartoRobson/Interruptor7/Comando";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub8 = "ESP32/MinhaCasa/QuartoRobson/Interruptor8/Comando";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub9  = "ESP32/MinhaCasa/QuartoRobson/Temperatura";					// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub10 = "ESP32/MinhaCasa/QuartoRobson/Umidade";						// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub11 = "ESP32/MinhaCasa/QuartoRobson/SensacaoTermica";				// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub12 = "ESP32/MinhaCasa/QuartoRobson/BMP180/Temperatura";			// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub13 = "ESP32/MinhaCasa/QuartoRobson/BMP180/PressaoAtmosferica/Real";		// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub14 = "ESP32/MinhaCasa/QuartoRobson/BMP180/PressaoAtmosferica/NivelMar";	// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub15 = "ESP32/MinhaCasa/QuartoRobson/BMP180/AltitudeReal";					// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
const char* pub16 = "ESP32/MinhaCasa/QuartoRobson/BMP180/AltitudeNivelMar";				// <-- Troque pro tópico do seu projeto, esse que está aí, é o meu
unsigned long lastMsgDHT = 0;     // Último tempo de leitura do DHT22
unsigned long lastMsgBMP180 = 0;  // Último tempo de leitura do BMP180

// Constantes para o sensor BMP180
const float pressaoNivelMar = 1012.0;  // Pressão ao nível do mar em sua localidade
const float altitudeNivelMar = 92.0;   // Altitude de referência do seu local

// Configurações do Broker MQTT
const char* BrokerMQTT  = "192.168.15.150"; // URL do broker MQTT, aqui você coloca o endereço do Broker, seja On Line ou Local
const char* LoginDoMQTT = "xxxxxxxxxxx";    // Usuário MQTT, não pesqueça de preencher com as credenciais do seu Broker
const char* SenhaMQTT   = "xxxxxxxxx";     // Senha MQTT, não esqueça de preencher com as credenciais do seu Broker
const int   MQTT_PORT   = 1883;              // Porta padrão do MQTT
const char* clientID    = "ESP32_Cliente";    // ID único do cliente

// =============== PROTÓTIPOS DE FUNÇÕES ===============
void setupWiFi();
void setupMQTT();
void handleMQTT();
void reconnectMQTT();
void checkWiFiConnection();
void readDHTSensor();
void readBMP180Sensor();
void readSensors();
void publishSensorData();
void publishRelayStates();

// Função para configurar o WiFi
void setupWiFi() {

  static bool configDone = false;
  unsigned long currentMillis = millis();

  // Controle de tentativas de conexão
  if (currentMillis - lastWifiRetryTime < WIFI_RETRY_INTERVAL) {
    return; // Ainda não é hora de tentar novamente
  }
  lastWifiRetryTime = currentMillis;

  // Configuração inicial
  if (!configDone) {
    Serial.println("Configurando rede...");
    // Tenta configurar IP fixo
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
      Serial.println("[ERRO] Falha ao configurar IP fixo!");
    } else {
      Serial.println("IP fixo configurado com sucesso!");
    }
    WiFi.begin(ssid, password);
    Serial.println("Conectando ao WiFi...");
    configDone = true;
  }

  // Controle do LED de status
  if (currentMillis - lastWifiBlinkTime >= WIFI_BLINK_INTERVAL) {
    lastWifiBlinkTime = currentMillis;
    digitalWrite(wifiLed, !digitalRead(wifiLed));
  }

  // Verifica status da conexão
  if (WiFi.status() == WL_CONNECTED && !wifiConnected) {
    digitalWrite(wifiLed, HIGH); // LED aceso quando conectado
    wifiConnected = true;
    Serial.println("\n==============================");
    Serial.println("WiFi CONECTADO!");
    Serial.print("IP do ESP32: ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Máscara de Sub-rede: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("DNS 1: ");
    Serial.println(WiFi.dnsIP(0));
    Serial.print("DNS 2: ");
    Serial.println(WiFi.dnsIP(1));
    Serial.println("==============================");
  } else if (WiFi.status() != WL_CONNECTED) {
    wifiConnected = false;
  }
}

// Função para configurar o MQTT
void setupMQTT() {
  mqttClient.setServer(BrokerMQTT, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  // Configura o keep alive do MQTT para 30 segundos (definido em MQTTParametros.h)
  mqttClient.setKeepAlive(MQTT_KEEPALIVE);
}

// Função para reconectar ao MQTT
void reconnectMQTT() {
  if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
    Serial.println("Tentando conectar ao MQTT...");
    if (mqttClient.connect(clientID, LoginDoMQTT, SenhaMQTT)) {
      Serial.println("==============================");
      Serial.println("MQTT CONECTADO!");
      Serial.print("IP do ESP32: ");
      Serial.println(WiFi.localIP());
      Serial.println("==============================");
      mqttConnected = true;
      // Inscrição nos tópicos dos relés
      mqttClient.subscribe(sub0);
      mqttClient.subscribe(sub1);
      mqttClient.subscribe(sub2);
      mqttClient.subscribe(sub3);
      mqttClient.subscribe(sub4);
      mqttClient.subscribe(sub5);
      mqttClient.subscribe(sub6);
      mqttClient.subscribe(sub7);
      mqttClient.subscribe(sub8);
      // Publica estado inicial dos relés
      publishRelayStates();
    } else {
      Serial.print("Falha na conexão MQTT, rc=");
      Serial.println(mqttClient.state());
      mqttConnected = false;
    }
  }
}

// Função de callback para mensagens MQTT recebidas
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String messageTemp;
  
  for (unsigned int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }
  
  Serial.print("Mensagem recebida [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(messageTemp);
  // Controle dos relés
  if (strcmp(topic, sub0) == 0) {
    // Ligar/Desligar todos
    bool estado = (messageTemp == "1");
    for (int i = RelayPin1; i <= RelayPin8; i++) {
      digitalWrite(i, !estado);  // Inverte o estado pois os relés são ativos em LOW
    }
    // Atualiza o estado de todos os relés
    RelayState1 = RelayState2 = RelayState3 = RelayState4 = 
    RelayState5 = RelayState6 = RelayState7 = RelayState8 = estado;
  }
  else if (strcmp(topic, sub1) == 0) {
    RelayState1 = (messageTemp == "1");
    digitalWrite(RelayPin1, !RelayState1);
  }
  else if (strcmp(topic, sub2) == 0) {
    RelayState2 = (messageTemp == "1");
    digitalWrite(RelayPin2, !RelayState2);
  }
  else if (strcmp(topic, sub3) == 0) {
    RelayState3 = (messageTemp == "1");
    digitalWrite(RelayPin3, !RelayState3);
  }
  else if (strcmp(topic, sub4) == 0) {
    RelayState4 = (messageTemp == "1");
    digitalWrite(RelayPin4, !RelayState4);
  }
  else if (strcmp(topic, sub5) == 0) {
    RelayState5 = (messageTemp == "1");
    digitalWrite(RelayPin5, !RelayState5);
  }
  else if (strcmp(topic, sub6) == 0) {
    RelayState6 = (messageTemp == "1");
    digitalWrite(RelayPin6, !RelayState6);
  }
  else if (strcmp(topic, sub7) == 0) {
    RelayState7 = (messageTemp == "1");
    digitalWrite(RelayPin7, !RelayState7);
  }
  else if (strcmp(topic, sub8) == 0) {
    RelayState8 = (messageTemp == "1");
    digitalWrite(RelayPin8, !RelayState8);
  }
}

// Função para ler e publicar dados dos sensores
void readSensors() {
  unsigned long currentTimeDHT = millis();
  unsigned long currentTimeBMP180 = millis();

  // Leitura do DHT22 a cada 60 segundos
  if (currentTimeDHT - lastMsgDHT > DHT_READ_INTERVAL) {
    lastMsgDHT = currentTimeDHT;

    float temp_data = dht.readTemperature();
    float hum_data = dht.readHumidity();

    // Verifica se as leituras são válidas
    if (isnan(temp_data) || isnan(hum_data)) {
      Serial.println("Falha na leitura do sensor DHT22!");
      return;
    }

    // Processa dados de temperatura e umidade
    dtostrf(temp_data, 6, 2, str_temp_data);
    dtostrf(hum_data, 6, 2, str_hum_data);

    // Processa temperatura em Fahrenheit e sensação térmica
    float tempF_data = dht.readTemperature(true);
    if (!isnan(tempF_data)) {
      dtostrf(tempF_data, 6, 2, str_tempF_data);
      
      float tempterm_data = dht.computeHeatIndex(tempF_data, hum_data);
      if (!isnan(tempterm_data)) {
        tempterm_data = dht.convertFtoC(tempterm_data);
        dtostrf(tempterm_data, 6, 2, str_tempterm_data);
        mqttClient.publish(pub11, str_tempterm_data);
      }
    }

    // Publica dados do DHT22
    mqttClient.publish(pub9, str_temp_data);
    mqttClient.publish(pub10, str_hum_data);

    // Debug no serial
    Serial.println("\n=== Leitura DHT22 ===");
    Serial.print("Temperatura: "); Serial.print(str_temp_data); Serial.println(" °C");
    Serial.print("Umidade: "); Serial.print(str_hum_data); Serial.println(" %");
    Serial.print("Sensação Térmica: "); Serial.print(str_tempterm_data); Serial.println(" °C");
  }

  // Leitura do BMP180 a cada 120 segundos
  if (currentTimeBMP180 - lastMsgBMP180 > BMP_READ_INTERVAL) {
    lastMsgBMP180 = currentTimeBMP180;
    char buffer[10];

    // NÃO chame bmp.begin() aqui!
    float pressure = bmp.readPressure();
    if (pressure != 0) {
      // Pressão atual
      dtostrf(pressure / 100.0, 2, 2, buffer);
      mqttClient.publish(pub13, buffer); // pub13: Pressão real

      // Pressão ao nível do mar
      float seaLevelPressure = bmp.readSealevelPressure(pressaoNivelMar);
      if (seaLevelPressure != 0) {
        dtostrf(seaLevelPressure / 100.0, 2, 2, buffer);
        mqttClient.publish(pub14, buffer); // pub14: Pressão ao nível do mar
      }

      // Altitude
      float altitudeReal = bmp.readAltitude(pressaoNivelMar * 100);
      if (!isnan(altitudeReal)) {
        // Publica altitude real
        dtostrf(altitudeReal, 2, 2, buffer);
        mqttClient.publish(pub15, buffer); // pub15: Altitude real

        // Calcula e publica altitude total
        altitude = altitudeReal + altitudeNivelMar;
        dtostrf(altitude, 2, 2, buffer);
        mqttClient.publish(pub16, buffer); // pub16: Altitude total

        // Debug no serial
        Serial.println("\n=== Leitura BMP180 ===");
        Serial.print("Pressão: "); Serial.print(pressure / 100.0); Serial.println(" hPa");
        Serial.print("Pressão nível do mar: "); Serial.print(seaLevelPressure / 100.0); Serial.println(" hPa");
        Serial.print("Altitude real: "); Serial.print(altitudeReal); Serial.println(" m");
        Serial.print("Altitude total: "); Serial.print(altitude); Serial.println(" m");
      }
    } else {
      Serial.println("Erro na leitura do sensor BMP180");
    }
  }
}

// Função para publicar dados dos sensores
void publishSensorData() {
  // Publicar dados do DHT22
  mqttClient.publish(pub9, str_temp_data);
  mqttClient.publish(pub10, str_hum_data);
  mqttClient.publish(pub11, str_tempterm_data);
  
  // Publicar dados do BMP180
  char tempStr[10];
  dtostrf(temperature, 6, 2, tempStr);
  mqttClient.publish(pub12, tempStr); // Temperatura BMP180
  
  dtostrf(pressure / 100.0, 6, 2, tempStr);
  mqttClient.publish(pub13, tempStr); // Pressão real BMP180
  
  // Pressão ao nível do mar (correta)
  float seaLevelPressure = bmp.readSealevelPressure(pressaoNivelMar);
  dtostrf(seaLevelPressure / 100.0, 6, 2, tempStr);
  mqttClient.publish(pub14, tempStr); // Pressão ao nível do mar BMP180
  
  // Altitude real
  float altitudeReal = bmp.readAltitude(pressaoNivelMar * 100);
  dtostrf(altitudeReal, 6, 2, tempStr);
  mqttClient.publish(pub15, tempStr); // Altitude real BMP180
  
  // Altitude em relação ao nível do mar
  dtostrf(altitude, 6, 2, tempStr);
  mqttClient.publish(pub16, tempStr); // Altitude total BMP180
}

// Função para publicar estado dos relés
void publishRelayStates() {
  mqttClient.publish(pub1, RelayState1 ? "1" : "0");
  mqttClient.publish(pub2, RelayState2 ? "1" : "0");
  mqttClient.publish(pub3, RelayState3 ? "1" : "0");
  mqttClient.publish(pub4, RelayState4 ? "1" : "0");
  mqttClient.publish(pub5, RelayState5 ? "1" : "0");
  mqttClient.publish(pub6, RelayState6 ? "1" : "0");
  mqttClient.publish(pub7, RelayState7 ? "1" : "0");
  mqttClient.publish(pub8, RelayState8 ? "1" : "0");
}

// =============== PROTÓTIPOS DE FUNÇÕES ===============
void setupWiFi();
void setupMQTT();
void handleMQTT();
void reconnectMQTT();
void checkWiFiConnection();
void readDHTSensor();
void readBMP180Sensor();
void readSensors();
void publishSensorData();
void publishRelayStates();

// =============== INÍCIO ===============
void setup() {

  // Cria task de conexões (WiFi/MQTT) no core 0
  xTaskCreatePinnedToCore(
    TaskConexoes,   // Função da task
    "TaskConexoes",// Nome
    4096,           // Stack size
    NULL,           // Param
    1,              // Prioridade
    NULL,           // Handle
    0               // Core 0
  );

  // Cria task de sensores/relés no core 1
  xTaskCreatePinnedToCore(
    TaskSensores,
    "TaskSensores",
    8192,
    NULL,
    1,
    NULL,
    1 // Core 1
  );
  // Inicialização da comunicação serial
  Serial.begin(115200);
  Serial.println("\nIniciando o sistema...");

  // Configura o WiFi
  setupWiFi();

  // Aguarda conexão WiFi antes de iniciar OTA
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi conectado!");
  Serial.println(WiFi.localIP());

  // Inicializa OTA somente após WiFi estar pronto
  setupOTA("ESP32-Automacao");
  
  // Configuração dos pinos dos relés como saída
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  pinMode(RelayPin3, OUTPUT);
  pinMode(RelayPin4, OUTPUT);
  pinMode(RelayPin5, OUTPUT);
  pinMode(RelayPin6, OUTPUT);
  pinMode(RelayPin7, OUTPUT);
  pinMode(RelayPin8, OUTPUT);
  pinMode(wifiLed, OUTPUT);

  // Inicializa todos os relés como desligados
  digitalWrite(RelayPin1, HIGH);
  digitalWrite(RelayPin2, HIGH);
  digitalWrite(RelayPin3, HIGH);
  digitalWrite(RelayPin4, HIGH);
  digitalWrite(RelayPin5, HIGH);
  digitalWrite(RelayPin6, HIGH);
  digitalWrite(RelayPin7, HIGH);
  digitalWrite(RelayPin8, HIGH);
  digitalWrite(wifiLed, LOW); // LED WiFi começa desligado

  // Inicializa o sensor DHT
  dht.begin();
  
  // Inicializa o I2C e o sensor BMP180
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bmp.begin()) {
    Serial.println("Não foi possível encontrar o sensor BMP180!");
    while (1) { }
  }

  // Configura MQTT
  setupMQTT();

}

// =============== EXECUÇÃO ===============
void loop() {
  ArduinoOTA.handle();
  vTaskDelay(10 / portTICK_PERIOD_MS); // Pequeno delay para não travar o core
}

// Task para conexões (WiFi/MQTT) - Core 0
void TaskConexoes(void *pvParameters) {
  (void) pvParameters;
  for(;;) {
    unsigned long currentMillis = millis();
    // WiFi
    if (currentMillis - lastWifiCheckTime >= WIFI_RETRY_INTERVAL) {
      lastWifiCheckTime = currentMillis;
      if (WiFi.status() != WL_CONNECTED) {
        wifiConnected = false;
        digitalWrite(wifiLed, LOW);
        setupWiFi();
      }
    }
    // MQTT
    if (!mqttClient.connected()) {
      mqttConnected = false;
      if (currentMillis - lastMqttReconnectAttempt >= RECONNECT_INTERVAL) {
        lastMqttReconnectAttempt = currentMillis;
        reconnectMQTT();
      }
    }
    mqttClient.loop();
    vTaskDelay(10 / portTICK_PERIOD_MS); // Pequeno delay para não travar o core
  }
}

// Task para sensores e relés - Core 1
void TaskSensores(void *pvParameters) {
  (void) pvParameters;
  for(;;) {
    // Lê os sensores (a função readSensors já tem controle de tempo interno)
    readSensors();
    // Atualiza estado dos relés se necessário
    unsigned long currentMillis = millis();
    if (mqttConnected && (currentMillis - lastRelayUpdate >= 5000)) {
      lastRelayUpdate = currentMillis;
      publishRelayStates();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
