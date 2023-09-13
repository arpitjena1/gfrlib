#pragma once
#include "main.h"

namespace triballControl {
enum Mode {
  reloading = 1,
  idle = 2,
  fire = 3,
  intake = 4,
  outtake = 5,  
};
int detected;
double hue;
int hold;
bool r1pressed = false;

void initialize();
extern void fireCata();
extern void intake();
extern void outtake();
Mode getMode();
}; // namespace discControl