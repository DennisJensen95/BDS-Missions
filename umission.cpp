/***************************************************************************
 *   Copyright (C) 2016-2020 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <sys/time.h>
#include <cstdlib>

#include "umission.h"
#include "utime.h"
#include "ulibpose2pose.h"

UMission::UMission(UBridge *regbot, UCamera *camera)
{
  cam = camera;
  bridge = regbot;
  threadActive = 100;
  // initialize line list to empty
  for (int i = 0; i < missionLineMax; i++)
  { // add to line list
    lines[i] = lineBuffer[i];
    // terminate c-strings strings - good practice, but not needed
    lines[i][0] = '\0';
  }
  // start mission thread
  th1 = new thread(runObj, this);
}

UMission::~UMission()
{
  printf("Mission class destructor\n");
}

void UMission::run()
{
  while (not active and not th1stop)
    usleep(100000);
  //   printf("UMission::run:  active=%d, th1stop=%d\n", active, th1stop);
  if (not th1stop)
    runMission();
  printf("UMission::run: mission thread ended\n");
}

void UMission::printStatus()
{
  printf("# ------- Mission ----------\n");
  printf("# active = %d, finished = %d\n", active, finished);
  printf("# mission part=%d, in state=%d\n", mission, missionState);
}

/**
 * Initializes the communication with the robobot_bridge and the REGBOT.
 * It further initializes a (maximum) number of mission lines 
 * in the REGBOT microprocessor. */
void UMission::missionInit()
{ // stop any not-finished mission
  bridge->send("robot stop\n");
  // clear old mission
  bridge->send("robot <clear\n");
  //
  // add new mission with 3 threads
  // one (100) starting at event 30 and stopping at event 31
  // one (101) starting at event 31 and stopping at event 30
  // one (  1) used for idle and initialisation of hardware
  // the mission is started, but staying in place (velocity=0, so servo action)
  //
  bridge->send("robot <add thread=1\n");
  // Irsensor should be activated a good time before use
  // otherwise first samples will produce "false" positive (too short/negative).
  bridge->send("robot <add irsensor=1,vel=0:dist<0.2\n");
  //
  // alternating threads (100 and 101, alternating on event 30 and 31 (last 2 events)
  bridge->send("robot <add thread=100,event=30 : event=31\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    // are to be replaced with real mission
    // NB - hereafter no lines can be added to these threads, just modified
    bridge->send("robot <add vel=0 : time=0.1\n");
  //
  bridge->send("robot <add thread=101,event=31 : event=30\n");
  for (int i = 0; i < missionLineMax; i++)
    // send placeholder lines, that will never finish
    bridge->send("robot <add vel=0 : time=0.1\n");
  usleep(10000);
  //
  //
  // send subscribe to bridge
  bridge->pose->subscribe();
  bridge->edge->subscribe();
  bridge->motor->subscribe();
  bridge->event->subscribe();
  bridge->joy->subscribe();
  bridge->motor->subscribe();
  bridge->info->subscribe();
  bridge->irdist->subscribe();
  bridge->imu->subscribe();
  usleep(10000);
  // there maybe leftover events from last mission
  bridge->event->clearEvents();
}

void UMission::sendAndActivateSnippet(char **missionLines, int missionLineCnt)
{
  // Calling sendAndActivateSnippet automatically toggles between thread 100 and 101.
  // Modifies the currently inactive thread and then makes it active.
  const int MSL = 100;
  char s[MSL];
  int threadToMod = 101;
  int startEvent = 31;
  // select Regbot thread to modify
  // and event to activate it
  if (threadActive == 101)
  {
    threadToMod = 100;
    startEvent = 30;
  }
  if (missionLineCnt > missionLineMax)
  {
    printf("# ----------- error - too many lines ------------\n");
    printf("# You tried to send %d lines, but there is buffer space for %d only!\n", missionLineCnt, missionLineMax);
    printf("# set 'missionLineMax' to a higher number in 'umission.h' about line 57\n");
    printf("# (not all lines will be send)\n");
    printf("# -----------------------------------------------\n");
    missionLineCnt = missionLineMax;
  }
  // send mission lines using '<mod ...' command
  for (int i = 0; i < missionLineCnt; i++)
  { // send lines one at a time
    if (strlen((char *)missionLines[i]) > 0)
    { // send a modify line command
      snprintf(s, MSL, "<mod %d %d %s\n", threadToMod, i + 1, missionLines[i]);
      bridge->send(s);
    }
    else
      // an empty line will end code snippet too
      break;
  }
  // let it sink in (10ms)
  usleep(10000);
  // Activate new snippet thread and stop the other
  snprintf(s, MSL, "<event=%d\n", startEvent);
  bridge->send(s);
  // save active thread number
  threadActive = threadToMod;
}

//////////////////////////////////////////////////////////

/**
 * Thread for running the mission(s)
 * All missions segments are called in turn based on mission number
 * Mission number can be set at parameter when starting mission command line.
 * 
 * The loop also handles manual override for the gamepad, and resumes
 * when manual control is released.
 * */
void UMission::runMission()
{ /// current mission number
  mission = fromMission;
  int missionOld = mission;
  bool regbotStarted = false;
  /// end flag for current mission
  bool ended = false;
  /// manuel override - using gamepad
  bool inManual = false;
  /// debug loop counter
  int loop = 0;
  // keeps track of mission state
  missionState = 0;
  int missionStateOld = missionState;
  // fixed string buffer
  const int MSL = 120;
  char s[MSL];
  /// initialize robot mission to do nothing (wait for mission lines)
  missionInit();
  /// start (the empty) mission, ready for mission snippets.
  bridge->send("start\n"); // ask REGBOT to start controlled run (ready to execute)
  bridge->send("oled 3 waiting for REGBOT\n");
  ///
  for (int i = 0; i < 3; i++)
  {
    if (not bridge->info->isHeartbeatOK())
    { // heartbeat should come at least once a second
      sleep(1);
    }
  }
  if (not bridge->info->isHeartbeatOK())
  { // heartbeat should come at least once a second
    system("espeak \"Oops, no usable connection with robot.\" -ven+f4 -s130 -a60 2>/dev/null &");
    bridge->send("oled 3 Oops: Lost REGBOT!");
    printf("# ---------- error ------------\n");
    printf("# No heartbeat from robot. Bridge or REGBOT is stuck\n");
    printf("# You could try restart ROBOBOT bridge ('b' from mission console) \n");
    printf("# -----------------------------\n");
    stop();
  }
  /// loop in sequence every mission until they report ended
  while (not finished and not th1stop)
  { // stay in this mission loop until finished
    loop++;
    // test for manuel override (joy is short for joystick or gamepad)
    if (bridge->joy->manual)
    { // just wait, do not continue mission
      usleep(20000);
      if (not inManual)
        system("espeak \"Mission paused.\" -ven+f4 -s130 -a40 2>/dev/null &");
      inManual = true;
      bridge->send("oled 3 GAMEPAD control\n");
    }
    else
    { // in auto mode
      if (not regbotStarted)
      { // wait for start event is received from REGBOT
        // - in response to 'bot->send("start\n")' earlier
        if (bridge->event->isEventSet(33))
        { // start mission (button pressed)
          //           printf("Mission::runMission: starting mission (part from %d to %d)\n", fromMission, toMission);
          regbotStarted = true;
        }
      }
      else
      { // mission in auto mode
        if (inManual)
        { // just entered auto mode, so tell.
          inManual = false;
          system("espeak \"Mission resuming.\" -ven+f4 -s130 -a40 2>/dev/null &");
          bridge->send("oled 3 running AUTO\n");
        }
        switch (mission)
        {
        case 1: // running auto mission
          ended = mission1(missionState);
          break;
        case 2:
          ended = mission2(missionState);
          break;
        case 3:
          ended = mission3(missionState);
          break;
        case 4:
          ended = mission4(missionState);
          break;
        default:
          // no more missions - end everything
          finished = true;
          break;
        }
        if (ended)
        { // start next mission part in state 0
          mission++;
          ended = false;
          missionState = 0;
        }
        // show current state on robot display
        if (mission != missionOld or missionState != missionStateOld)
        { // update small O-led display on robot - when there is a change
          UTime t;
          t.now();
          snprintf(s, MSL, "oled 4 mission %d state %d\n", mission, missionState);
          bridge->send(s);
          if (logMission != NULL)
          {
            fprintf(logMission, "%ld.%03ld %d %d\n",
                    t.getSec(), t.getMilisec(),
                    missionOld, missionStateOld);
            fprintf(logMission, "%ld.%03ld %d %d\n",
                    t.getSec(), t.getMilisec(),
                    mission, missionState);
          }
          missionOld = mission;
          missionStateOld = missionState;
        }
      }
    }
    //
    // check for general events in all modes
    // gamepad buttons 0=green, 1=red, 2=blue, 3=yellow, 4=LB, 5=RB, 6=back, 7=start, 8=Logitech, 9=A1, 10 = A2
    // gamepad axes    0=left-LR, 1=left-UD, 2=LT, 3=right-LR, 4=right-UD, 5=RT, 6=+LR, 7=+-UD
    // see also "ujoy.h"
    if (bridge->joy->button[BUTTON_RED])
    { // red button -> save image
      if (not cam->saveImage)
      {
        printf("UMission::runMission:: button 1 (red) pressed -> save image\n");
        cam->saveImage = true;
      }
    }
    if (bridge->joy->button[BUTTON_YELLOW])
    { // yellow button -> make ArUco analysis
      if (not cam->doArUcoAnalysis)
      {
        printf("UMission::runMission:: button 3 (yellow) pressed -> do ArUco\n");
        cam->doArUcoAnalysis = true;
      }
    }
    // are we finished - event 0 disables motors (e.g. green button)
    if (bridge->event->isEventSet(0))
    { // robot say stop
      finished = true;
      printf("Mission:: insist we are finished\n");
    }
    else if (mission > toMission)
    { // stop robot
      // make an event 0
      bridge->send("stop\n");
      // stop mission loop
      finished = true;
    }
    // release CPU a bit (10ms)
    usleep(10000);
  }
  bridge->send("stop\n");
  snprintf(s, MSL, "espeak \"%s finished.\"  -ven+f4 -s130 -a12  2>/dev/null &", bridge->info->robotname);
  system(s);
  printf("Mission:: all finished\n");
  bridge->send("oled 3 finished\n");
}

////////////////////////////////////////////////////////////

/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission1(int &state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  switch (state)
  {
  case 0:
    // tell the operatior what to do
    printf("# press green to start.\n");
    system("espeak \"press green to start\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 press green to start");
    state++;
    break;
  case 1:
    // -------------------------
    //state = 10; //debug without controller
    // -------------------------
    if (bridge->joy->button[BUTTON_GREEN])
      state = 10;
    break;
  case 10: // follow black line for 0.5 m at a lower velocity
    snprintf(lines[0], MAX_LEN, "vel=0.2, acc=1, edgel=-1, white=1 : time=1");
    // increase velocity and follow line until right IR sensor detects the guillotine gate
    // gates are 45 cm wide and robot is approx 32 cm wide giving ~7 cm on either side
    snprintf(lines[1], MAX_LEN, "vel=0.6, acc=1, edgel=-1, white=1 : ir1 < 0.15");
    //drive 25 cm to steer clear of the gate
    snprintf(lines[2], MAX_LEN, "vel=0.5, acc=1, edgel=-1, white=0 : dist=0.20");
    // stop and create an event when arrived at this point
    snprintf(lines[3], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[4], MAX_LEN, ": dist=1");
    // send the 4 lines to the REGBOT
    sendAndActivateSnippet(lines, 5);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet 1\n", state);
    system("espeak \"code snippet 1.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet 1");
    //
    // go to wait for finished
    state = 11;
    featureCnt = 0;
    break;
  case 11:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 999;
    }
    break;
  case 999:
  default:
    printf("mission 1 ended \n");
    bridge->send("oled 5 \"mission 1 ended.\"");
    finished = true;
    break;
  }
  return finished;
}


/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission2(int &state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  switch (state)
  {
  case 0:
    // continue driving along the line until the first gate is found
    snprintf(lines[0], MAX_LEN, "vel=0.5, acc=1.0, edgel=1.0, white=1: ir1<0.15");
    // drive slowly towards the intersection
    snprintf(lines[1], MAX_LEN, "vel=0.2,edgel=1,white=1: xl>6");
    // pause
    snprintf(lines[2], MAX_LEN, "vel=0: time=0.5");
    // turn onto see saw
    snprintf(lines[3], MAX_LEN, "vel=0.2, acc=0.5, tr=0.01: turn=90.0");
    // pause
    snprintf(lines[4], MAX_LEN, "vel=0: time=0.5");
    // create event 1
    snprintf(lines[5], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[6], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, 7);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet 2\n", state);
    system("espeak \"code snippet 2.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet 2");
    //
    // go to wait for finished
    state = 11;
    featureCnt = 0;
    break;
  case 11:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 999;
    }
    break;
  case 999:
  default:
    printf("mission 2 ended \n");
    bridge->send("oled 5 \"mission 2 ended.\"");
    finished = true;
    break;
  }
  return finished;
}


/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission3(int &state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  switch (state)
  {
  case 0:
    // drive slowly until the gate at the end of the see saw is reached
    snprintf(lines[0], MAX_LEN, "vel=0.15, acc=1.0, edger=0.0, white=1: ir1<0.15");
    // pause after reaching the ground
    snprintf(lines[1], MAX_LEN, "vel=0.0: time=2.0");
    // drive away from see saw
    snprintf(lines[2], MAX_LEN, "vel=0.3: dist=0.6");
    // create event 1
    snprintf(lines[3], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[4], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, 5);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet 3\n", state);
    system("espeak \"code snippet 3.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet 3");
    //
    // go to wait for finished
    state = 11;
    featureCnt = 0;
    break;
  case 11:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 999;
    }
    break;
  case 999:
  default:
    printf("mission 3 ended \n");
    bridge->send("oled 5 \"mission 3 ended.\"");
    finished = true;
    break;
  }
  return finished;
}

/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission4(int & state)
{
  bool finished = false;
  switch (state)
  {
    case 999:
    default:
      printf("mission 4 ended\n");
      bridge->send("oled 5 mission 4 ended.");
      finished = true;
      break;
  }
  return finished;
}

void UMission::openLog()
{
  // make logfile
  const int MNL = 100;
  char date[MNL];
  char name[MNL];
  UTime appTime;
  appTime.now();
  appTime.getForFilename(date);
  // construct filename ArUco
  snprintf(name, MNL, "log_mission_%s.txt", date);
  logMission = fopen(name, "w");
  if (logMission != NULL)
  {
    const int MSL = 50;
    char s[MSL];
    fprintf(logMission, "%% Mission log started at %s\n", appTime.getDateTimeAsString(s));
    fprintf(logMission, "%% Start mission %d end mission %d\n", fromMission, toMission);
    fprintf(logMission, "%% 1  Time [sec]\n");
    fprintf(logMission, "%% 2  mission number.\n");
    fprintf(logMission, "%% 3  mission state.\n");
  }
  else
    printf("#UCamera:: Failed to open image logfile\n");
}

void UMission::closeLog()
{
  if (logMission != NULL)
  {
    fclose(logMission);
    logMission = NULL;
  }
}
