#include "navigation.h"


#ifndef PATH_OPTIONS_H
#define PATH_OPTIONS_H

float Run1DTimeOptimalControl(float dist_to_go, float current_speed, const navigation::NavigationParams& nav_params);

#endif  // PATH_OPTIONS_H