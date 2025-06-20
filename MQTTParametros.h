// =================== PARÂMETROS MQTT (Retain, QoS, KeepAlive) ===================
#ifndef MQTTPARAMETROS_H
#define MQTTPARAMETROS_H

// KeepAlive (em segundos)
#define MQTT_KEEPALIVE 30

// Parâmetros para os Relés
const bool retainReles[8] = {true, true, true, true, true, true, true, true}; // Pode alterar individualmente
const uint8_t qosReles[8] = {2, 2, 2, 2, 2, 2, 2, 2}; // Pode alterar individualmente

// Parâmetros para o DHT22
const bool retainDHT22[3] = {false, false, false}; // [0]=temp, [1]=umidade, [2]=sensação térmica
const uint8_t qosDHT22[3] = {0, 0, 0};

// Parâmetros para o BMP180
const bool retainBMP180[4] = {false, false, false, false}; // [0]=temp, [1]=pressão real, [2]=pressão mar, [3]=altitude
const uint8_t qosBMP180[4] = {0, 0, 0, 0};

#endif // MQTTPARAMETROS_H
