#include <Arduino.h>
#include "pins.h"
#include "ihm.h"
#include "sensors.h"
#include "actuators.h"
#include "motion.h"
#include "match.h"

void waitStart();
void datumPosition(int robotNumber, int teamColor);
void match();
void approachStartZone();
void strategiePAMI();

void setup()
{

  initBuzzer();
  initIHM();
  initSensor();
  initMotion();
  initActuators();

  drawSplashScreen();
  // pairingScreen();
  drawBackLcd();

  disableMotors();
  armsDown();
  selectMelody(getRobotNumber());

  //while(1) readSensors(true); // TODO : Test sensors

  waitStart();
}

void loop()
{
  updateMatchTime();
  match();
}

void waitStart()
{
  // Attendre que la tirette ne soit plus présente
  infoLCD("Remove Tirette");
  while (getTirette())
  {
    delay(250);
    checkColorTeam();
  }
  playTirette();
  // Attendre que la tirette soit insérée
  infoLCD("Insert Tirette");
  while (!getTirette())
  {
    delay(500);
    checkColorTeam();
  }
  playTirette();
  // Datum position du PAMI
  delay(1000);
  datumPosition(getRobotNumber(), getTeamColor());
  setRobotState(READY);
  infoLCD("Robot Ready");
  delay(2000);
  // Attendre que la tirette soit bien insérée pour éviter les faux-départs
  infoLCD("Insert Tirette");
  while (!getTirette())
    delay(500);
  playTirette();
  // Attendre que la tirette soit retirée pour débuter le match
  infoLCD("Wait Start");
  while (getTirette())
    delay(250);
  playTirette();
  // Le match commence
  setRobotState(MATCH_STARTED);
  infoLCD("Go Match !");
  // Démarrage du compteur !
  startMatch();
}



void datumPosition(int robotNumber, int teamColor)
{
  enableMotors();
  // Datum at low Speed
  setMaxSpeed(DATUM_SPEED);
  setAcceleration(DATUM_ACCELERATION);

  if (teamColor == TEAM_BLUE)
  {
    // Datum Y
    setCurrentY(CENTER_POSITION_MM);
    setCurrentRot(90);

    // Datum X
    if      (robotNumber == 0) setCurrentX(3000-508);
    else if (robotNumber == 1) setCurrentX(3000-400);
    else if (robotNumber == 2) setCurrentX(3000-292);
    else if (robotNumber == 3) setCurrentX(3000-184);
    else
      debug("ERROR robot number");
  }
  else if (teamColor == TEAM_YELLOW)
  {

    // Datum Y
    setCurrentY(CENTER_POSITION_MM);
    setCurrentRot(90);

    // Datum X
    if      (robotNumber == 0) setCurrentX(508);
    else if (robotNumber == 1) setCurrentX(400);
    else if (robotNumber == 2) setCurrentX(292);
    else if (robotNumber == 3) setCurrentX(184);
    else
      debug("ERROR robot number");
  }

  setMaxSpeed(MAX_SPEED);
  setAcceleration(MAX_ACCELERATION);
}

void match()
{
  if (getMatchState() == MAIN_BOT_AWAY)
  {
    infoLCD("Main Bot Away");
    enableMotors();
    approachStartZone();
    setMatchState(PAMI_WAIT);
  }
  else if (getMatchState() == PAMI_RUN)
  {
    infoLCD("Match Running");
    enableMotors();
    strategiePAMI();
    setMatchState(PAMI_STOP);
    infoLCD("Match Stopped");
  }
  else if (getMatchState() == PAMI_STOP)
  {
    disableMotors(); // Desactive les moteurs
    infoLCD("Fin Match !");
    while (1)
      armsFiesta(); // Fin de match
  }
  else
  {
    disableMotors(); // Desactive les moteurs
  }
}

void approachStartZone(){
  float safeY = 375;
  if (getTeamColor() == TEAM_BLUE)
  {
    if      (getRobotNumber() == 0) goTo(getCurrentX(), safeY);
    else if (getRobotNumber() == 1) goTo(getCurrentX(), safeY);
    else if (getRobotNumber() == 2) goTo(getCurrentX(), safeY);
    else if (getRobotNumber() == 3) goTo(getCurrentX(), safeY);
  }
  else if (getTeamColor() == TEAM_YELLOW)
  {
    if      (getRobotNumber() == 0) goTo(getCurrentX(), safeY);
    else if (getRobotNumber() == 1) goTo(getCurrentX(), safeY);
    else if (getRobotNumber() == 2) goTo(getCurrentX(), safeY);
    else if (getRobotNumber() == 3) goTo(getCurrentX(), safeY);
  }
}

void strategiePAMI()
{

  setOpponentChecking(true);
  if (getRobotNumber() == 0)
  {
    if (getTeamColor() == TEAM_BLUE)
    {
      goTo(3000-600, 600);
      goTo(3000-1425, 1128);
    }
    else
    {
      goTo(600, 600);
      goTo(1425, 1128);
    }
  }
  else if (getRobotNumber() == 1)
  {
    if (getTeamColor() == TEAM_BLUE)
    {
      pause(2000);
      goTo(3000-1400, 1820);
    }
    else
    {
      pause(2000);
      goTo(1400, 1820);
    }
  }
  else if (getRobotNumber() == 2)
  {
    if (getTeamColor() == TEAM_BLUE)
    {
      pause(4000);
      goTo(3000-623, 1820);
    }
    else
    {
      pause(4000);
      goTo(623, 1820);
    }
  }
  else if (getRobotNumber() == 3)
  {
    if (getTeamColor() == TEAM_BLUE)
    {
      pause(6000);
      goTo(3000-728, 1128);
    }
    else
    {
      pause(6000);
      goTo(728, 1128);
    }
  }
}