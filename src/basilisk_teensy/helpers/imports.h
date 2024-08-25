#pragma once

#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Arduino.h>
#include <Moteus.h>
#include <Wire.h>

namespace basilisk {
using namespace mjbots::moteus;
using PmCmd = PositionMode::Command;
using PmFmt = PositionMode::Format;
using QRpl = Query::Result;
using QFmt = Query::Format;
}  // namespace basilisk
