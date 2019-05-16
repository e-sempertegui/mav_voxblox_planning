#include "loco_planner/loco.h"
// #include <iostream>

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  loco_planner::Loco<10> loco(3);
  // cout << "loco_main.cpp IS BEING EXECUTED!!!! FIXED THE ARGUMENT"

  return 0;
}
