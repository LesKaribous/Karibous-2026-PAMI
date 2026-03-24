// 
#include "match.h"

long elapsedTime  = 0;
long startTime    = 0;

int matchState = MATCH_WAIT;

int getMatchState(){
    return matchState;
}
void setMatchState(int _state){
    matchState = _state;
}
void startMatch(){
    startTime = millis();
    matchState = MATCH_BEGIN;
}
void updateMatchTime(){
  if(getMatchState() == MATCH_BEGIN || getMatchState() == PAMI_RUN || getMatchState() == PAMI_WAIT){
    elapsedTime = millis() - startTime ;
    if(elapsedTime>= TIME_END_MATCH)
    {
        setMatchState(PAMI_STOP);
        debug("Match End");
    }
    else if(elapsedTime>= TIME_MAIN_BOT_AWAY && getMatchState() == MATCH_BEGIN)
    {
        setMatchState(MAIN_BOT_AWAY);
        debug("Main bot away");
    }
    else if(elapsedTime>= TIME_START_MATCH && getMatchState() == PAMI_WAIT )
    {
        setMatchState(PAMI_RUN);
        debug("Match Run");
    }
    else if(getMatchState() == MATCH_WAIT || getMatchState() == PAMI_WAIT)
    {
        infoLCD(String((TIME_START_MATCH-elapsedTime)/1000) + " sec");
        debug("Waiting...");
    }
  }
}
long getElapsedTime(){
    return elapsedTime ;
}
long getStartTime(){
    return startTime ;
}
void pause(long millisecondes) {
    unsigned long startPause = millis(); // Enregistre le temps de début de la pause

    while (millis() - startPause < millisecondes) {
        updateMatchTime(); // Met à jour l'état du match

        if (getMatchState() == PAMI_STOP) {
            debug("Pause interrompue : match terminé");
            break; // Sort de la pause si le match est terminé
        }

        // Ajout d'un petit délai non bloquant pour éviter de monopoliser le CPU
        delayMicroseconds(500);  
    }
}
