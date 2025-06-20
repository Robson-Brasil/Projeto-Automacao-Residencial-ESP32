#ifndef OTA_CONFIG_H
#define OTA_CONFIG_H

#include <ArduinoOTA.h>

void setupOTA(const char* hostname = "ESP32-xxxxx") {
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword("xxxxxxxxx");  //Cria uma senha pra proteção da atualização, e não esquece de trocar o hostname

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem";
    Serial.println("[OTA] Iniciando atualização: " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Atualização finalizada!");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progresso: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Erro[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Falha de autenticação");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Falha ao iniciar");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Falha de conexão");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Falha ao receber");
    else if (error == OTA_END_ERROR) Serial.println("Falha ao finalizar");
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] Pronto para atualização OTA!");
}

#endif // OTA_CONFIG_H
