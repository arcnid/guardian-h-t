#ifndef API_H
#define API_H

#include <ESP8266WebServer.h>
#include "../utils/utils.h"
#include <ArduinoJson.h>


//utility functions go here
void setupApiRoutes(ESP8266WebServer &server);

#endif