#pragma once

//#define COMM_WIFI  // comment out to use Serial

void commInit();
bool commRead(float &v, float &w);
