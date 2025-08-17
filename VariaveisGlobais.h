// =================== VARIÁVEIS GLOBAIS ===================
#ifndef VARIAVEISGLOBAIS_H
#define VARIAVEISGLOBAIS_H

// Estados dos relés
enum RelayStates
{
    RELAY_OFF = 0,
    RELAY_ON = 1
};

extern bool Todos;
extern bool RelayState1;
extern bool RelayState2;
extern bool RelayState3;
extern bool RelayState4;
extern bool RelayState5;
extern bool RelayState6;
extern bool RelayState7;
extern bool RelayState8;

extern char str_hum_data[7];
extern char str_temp_data[7];
extern char str_tempterm_data[7];
extern char str_tempF_data[7];
extern float temperature;
extern float pressure;
extern float altitude;
extern float altitudeTotal;
extern unsigned long lastMsgDHT;
extern unsigned long lastMsgBMP180;
extern unsigned long lastWifiRetryTime;
extern unsigned long lastWifiBlinkTime;
extern unsigned long lastWifiCheckTime;
extern unsigned long lastMqttReconnectAttempt;
extern unsigned long lastRelayUpdate;
extern bool wifiConnected;
extern bool mqttConnected;

#endif // VARIAVEISGLOBAIS_H
