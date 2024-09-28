#pragma once

#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Moteus.h>
#include <Smoothed.h>
#include <math.h>

#include <vector>

#include "do_you_want_debug.h"
#include "utils.h"

using namespace mjbots::moteus;
using PmCmd = PositionMode::Command;
using PmFmt = PositionMode::Format;
using QRpl = Query::Result;
using QFmt = Query::Format;
