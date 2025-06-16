// =================== CONFIGURAÇÃO MQTT ===================
#ifndef CONFIGMQTT_H
#define CONFIGMQTT_H

#include "LoginsSenhas.h"

void setupMQTT();
void reconnectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);

#endif // CONFIGMQTT_H
