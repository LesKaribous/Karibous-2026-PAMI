// match.h
#ifndef MATCH_H
#define MATCH_H

#include <Arduino.h>
#include "ihm.h"

#define TIME_MAIN_BOT_AWAY   5000 // Temps pour que le robot principal puisse s'éloigner de la zone de départ avant que les autres ne commencent à bouger
#define TIME_START_MATCH    85000  // TODO 85000
#define TIME_END_MATCH     100000   // TODO 100000

#define MATCH_WAIT 0
#define MATCH_BEGIN 1
#define PAMI_RUN 2
#define PAMI_STOP 3
#define MAIN_BOT_AWAY 4
#define PAMI_WAIT 5

int getMatchState();
void setMatchState(int _state);
void startMatch();
void updateMatchTime();
long getElapsedTime();
long getStartTime();
void pause(long millisecondes);

#endif // MATCH_H
