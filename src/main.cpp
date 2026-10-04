#include <Arduino.h>

#include "application.h"

namespace {
gps_logger::Application application;
}

void setup() { application.begin(); }

void loop() { application.tick(); }
