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
          //ended = mission1(missionState);
          ended = true;
          break;
        case 2:
          //ended = mission2(missionState);
          ended = true;
          break;
        case 3:
          ended = mission3(missionState);
          break;
        case 4:
          ended = mission4(missionState);
          break;
        case 5:
          ended = mission5(missionState);
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
    if (bridge->joy->button[BUTTON_GREEN])
      state = 10;
    break;
  case 10:
  {
    int line = 0;
    // follow white line for 2 ss at a lower velocity
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=1, edgel=-1, white=1 : time=2");
    // increase velocity and follow line until right IR sensor detects a hand waving
    snprintf(lines[line++], MAX_LEN, "vel=0.5, acc=1, edgel=-1, white=1 : ir2 < 0.15");
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
    // stop and create an event when arrived at this point
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 4 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet 1\n", state);
    system("espeak \"code snippet 1.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet 1");
    //
    // go to wait for finished
    state = 11;
    break;
  }
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
  {

    float heading = (bridge->pose->h)*180.0/M_PI;

    // printf("Heading = %f\n", heading);

    int line = 0;
    // turn 90 degrees to the left
    snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.6, head=%.1f: turn=90", heading+90.0);
    // drive while obstacle is in view 
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.8: ir1 > 0.5");
    // drive forward 30 cm
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.8: dist = 0.3");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet\n", state);
    system("espeak \"code snippet.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet");
    //
    // go to wait for finished
    state = 11;
    break;
  }
  case 11:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 12;
    }
    break;
  case 12:
  {
    float heading = (bridge->pose->h)*180.0/M_PI;

    // printf("Heading = %f\n", heading);

    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 90 degrees to the left
    snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.6, head=%.1f: turn=-90", heading-90.0);
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // drive until obstacle is in view 
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.8: ir1 < 0.3");
    // drive while obstacle is in view 
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.8: ir1 > 0.5");
    // drive forward 30 cm
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.8: dist = 0.3");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet\n", state);
    system("espeak \"code snippet.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet");
    //
    // go to wait for finished
    state = 13;
    break;
  }
  case 13:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 14;
    }
    break;
  case 14:
  {
    float heading = (bridge->pose->h)*180.0/M_PI;

    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 90 degrees to the right
    snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.6, head=%.1f: turn=-90", heading-90.0);
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet\n", state);
    system("espeak \"code snippet.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet");
    //
    // go to wait for finished
    state = 15;
    break;
  }
  case 15:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 20;
    }
    break;
  case 20:
  {
    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 90 degrees to the right
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.6: xl=20");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet\n", state);
    system("espeak \"code snippet.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet");
    //
    // go to wait for finished
    state = 21;
    break;
  }
  case 21:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 22;
    }
    break;
  case 22:
  {
    // float heading = (bridge->pose->h)*180.0/M_PI;

    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 90 degrees to the left
    //snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.6, head=%.1f: turn=90", heading+90.0);
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.6, tr=0.1: turn=90");
    // follow line
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=1, edgel=-1, white=1 : dist=0.5");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet\n", state);
    system("espeak \"code snippet.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet");

    // go to wait for finished
    state = 23;
    break;
  }
  case 23:
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
  case 0: // start
    caseCounter = 0;
    // tell the operatior what to do
    printf("# started mission 3.\n");
    system("espeak \"looking for ball\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 looking 4 ball");
    state = 10;
    break;
  case 10:
  {
    int line = 0;
    // raise arm
    snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-300, vservo=10");
    // open grabber
    snprintf(lines[line++], MAX_LEN, "servo=2, pservo=300, vservo=10");
    // wait 4 seconds
    snprintf(lines[line++], MAX_LEN, ": time = 4");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // wait for finished setting servo
    state = 11;
  }
  case 11: 
    // wait for event 1
    if (bridge->event->isEventSet(1))
    { // continue
      state = 12;
    }
    break;
  case 12: // start ball analysis
  {
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < (2 * 180 / M_PI))
    { // finished first drive and turnrate is zero'ish
      state = 20;
      // wait further 30ms - about one camera frame at 30 FPS
      usleep(35000);
      // start ball analysis
      printf("# started new ball analysis\n");
      cam->doFindBall = true;
    }
    break;
  }
  case 20:
    if (not cam->doFindBall) // ball processing finished
    {
      if (cam->ballFound == 0)
      { // found a single ball
        state = 30;
        // tell the operator
        printf("# case=%d found ball\n", state);
        system("espeak \"found ball.\" -ven+f4 -s130 -a5 2>/dev/null &");
        bridge->send("oled 5 found ball");
      }
      else
      { // start turning procedure
        if (caseCounter < 0){
          printf("%d\n", caseCounter);
          state = 21;
        }
        else{
          state = 999;
        }
      }
    }
    break;
  case 21:
  {
    float heading = (bridge->pose->h)*180.0/M_PI;

    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 30 degrees to the right
    snprintf(lines[line++], MAX_LEN, "topos=0, vel=0.1, acc=0.6, head=%.1f: turn=-40, time=2", heading-40.0);
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d turning robot\n", state);
    bridge->send("oled 5 turning...");
    //
    // go to wait for finished
    state = 22;
    break;
  }
  case 22:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      printf("increment counter...\n");
      caseCounter++;
      state = 12;
    }
    break;
  case 30: // move to ball
  {
    FindBall *v = cam->findBalls->returnDataPointer();

    float xm = v->ballPosition.at<float>(0,0);
    float ym = v->ballPosition.at<float>(0,1);
    float hm = v->ballAngle;

    // stop some distance in front of marker
    float dx = 0.35; // distance to stop in front of marker
    float dy = 0.1; // distance to the left of marker
    xm += - dx*cos(hm) + dy*sin(hm);
    ym += - dx*sin(hm) - dy*cos(hm);
    // limits
    float acc = 1.0; // max allowed acceleration - linear and turn
    float vel = 0.3; // desired velocity
    // set parameters
    // end at 0 m/s velocity
    UPose2pose pp4(xm, ym, hm, 0.0);
    printf("\n");
    // calculate turn-straight-turn (Angle-Line-Angle)  manoeuvre
    bool isOK = pp4.calculateALA(vel, acc);
    // use only if distance to destination is more than 3cm
    if (isOK and (pp4.movementDistance() > 0.03))
    { // a solution is found - and more that 3cm away.
      // debug print manoeuvre details
      pp4.printMan();
      printf("\n");
      // debug end
      int line = 0;
      if (pp4.initialBreak > 0.01)
      { // there is a starting straight part
        snprintf(lines[line++], MAX_LEN, "vel=%.3f,acc=%.1f :dist=%.3f", 
                  pp4.straightVel, acc, pp4.straightVel);
      }
      snprintf(lines[line++], MAX_LEN,   "vel=%.3f,tr=%.3f :turn=%.1f", 
                pp4.straightVel, pp4.radius1, pp4.turnArc1 * 180 / M_PI);
      snprintf(lines[line++], MAX_LEN,   ":dist=%.3f", pp4.straightDist);
      snprintf(lines[line++], MAX_LEN,   "tr=%.3f :turn=%.1f", 
                pp4.radius2, pp4.turnArc2 * 180 / M_PI);
      if (pp4.finalBreak > 0.01)
      { // there is a straight break distance
        snprintf(lines[line++], MAX_LEN,   "vel=0 : time=%.2f", 
                  sqrt(2*pp4.finalBreak));
      }
      snprintf(lines[line++], MAX_LEN,   "vel=0, event=2: dist=1");
      sendAndActivateSnippet(lines, line);
      // make sure event 2 is cleared
      bridge->event->isEventSet(2);
      //
      // debug
      for (int i = 0; i < line; i++)
      { // print sent lines
        printf("# line %d: %s\n", i, lines[i]);
      }
      // debug end
      // tell the operator
      printf("# Sent mission snippet to ball (%d lines)\n", line);
      //system("espeak \"code snippet to marker.\" -ven+f4 -s130 -a20 2>/dev/null &"); 
      bridge->send("oled 5 code to ball");
      // wait for movement to finish
      state = 31;
    }
    else
    { // no marker or already there
      printf("# No need to move, just %.2fm, frame %d\n", 
              pp4.movementDistance(), v->frameNumber);
      // look again for marker
      state = 11;
    }
    v->lock.unlock();
    
    break;
  }
  case 31: // check if movement is finished
    // wait for event 2 (send when finished driving)
    if (bridge->event->isEventSet(2))
    { // stop
      state = 40;
    }
    break;
  case 40:
  {
    int line = 0;
    // lower arm
    snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=480, vservo=10");
    // wait 2 seconds
    snprintf(lines[line++], MAX_LEN, ": time = 2");
    // close grabber
    snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-200, vservo=10");
    // wait 2 seconds
    snprintf(lines[line++], MAX_LEN, ": time = 2");
    // raise arm
    snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-300, vservo=10");
    // wait 2 seconds
    snprintf(lines[line++], MAX_LEN, ": time = 5");
    // lower arm
    snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=480, vservo=10");
    // wait 2 seconds
    snprintf(lines[line++], MAX_LEN, ": time = 2");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet 10\n", state);
    system("espeak \"code snippet 10.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet 10");
    
    // go to wait for finished
    state = 41;
    break;
  }
  case 41:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 999;
    }
    break;
  case 999: // end
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
 * THE RAMP PART 2 **NOT TESTED**
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission4(int &state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  switch (state)
  {
  case 0:
  {
    int line = 0;
    // yolo
    snprintf(lines[line++], MAX_LEN, "vel=0");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet 10\n", state);
    system("espeak \"code snippet 10.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet 10");
    //
    // go to wait for finished
    state = 11;
    featureCnt = 0;
    break;
  }
  case 11:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 999;
    }
    break;
  case 999:
  default:
    printf("mission 4 ended \n");
    bridge->send("oled 5 \"mission 4 ended.\"");
    finished = true;
    break;
  }
  return finished;
}

/**
 * Run mission
 * THE STAIRS
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission5(int &state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  switch (state)
  {
  case 0:
  {
    int line = 0;
    // yolo
    snprintf(lines[line++], MAX_LEN, "vel=0");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet 10\n", state);
    system("espeak \"code snippet 10.\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 code snippet 10");
    //
    // go to wait for finished
    state = 11;
    featureCnt = 0;
    break;
  }
  case 11:
    // wait for event 1 (send when finished driving first part)
    if (bridge->event->isEventSet(1))
    { // finished first drive
      state = 999;
    }
    break;
  case 999:
  default:
    printf("mission 6 ended \n");
    bridge->send("oled 5 \"mission 6 ended.\"");
    finished = true;
    break;
  }
  return finished;
}

/*
* //////////////////////////////////////////
* //////////////////////////////////////////
* ///////////// MISSIONS DONE //////////////
* //////////////////////////////////////////
* //////////////////////////////////////////
*/

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
