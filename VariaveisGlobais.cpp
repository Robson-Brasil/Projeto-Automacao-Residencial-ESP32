#include "VariaveisGlobais.h"

// Definições (sem "extern" aqui)
bool Todos = false;
bool RelayState1 = false;
bool RelayState2 = false;
bool RelayState3 = false;
bool RelayState4 = false;
bool RelayState5 = false;
bool RelayState6 = false;
bool RelayState7 = false;
bool RelayState8 = false;

char str_hum_data[7] = "";
char str_temp_data[7] = "";
char str_tempterm_data[7] = "";
char str_tempF_data[7] = "";
float temperature = 0.0f;
float pressure = 0.0f;
float altitude = 0.0f;
float altitudeTotal = 0.0f;
unsigned long lastMsgDHT = 0;
unsigned long lastMsgBMP180 = 0;
unsigned long lastWifiRetryTime = 0;
unsigned long lastWifiBlinkTime = 0;
unsigned long lastWifiCheckTime = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastRelayUpdate = 0;
bool wifiConnected = false;
bool mqttConnected = false;
