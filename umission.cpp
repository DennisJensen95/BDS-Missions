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
          //ended = true;
          break;
        case 2:
          ended = mission2(missionState);
          //ended = true;
          break;
        case 3:
          ended = mission3(missionState);
          //ended = true;
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
    // start log
    // system("lo pose hbt imu ir motor joy event cam aruco mission");
    // tell the operatior what to do
    printf("# press green to start.\n");
    system("espeak \"press green to start\" -ven+f4 -s130 -a5 2>/dev/null &");
    bridge->send("oled 5 press green to start");
    state++;
    break;
  case 1:
    if (bridge->joy->button[BUTTON_GREEN]){
      PlaySound("m1start.mp3");
      sleep(3);
      state = 10;
    }
    break;
  case 10:
  {
    int line = 0;
    // raise arm
    snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-300, vservo=10");
    // follow white line for 2 ss at a lower velocity
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=1, edgel=-1, white=1 : time=2");
    // increase velocity and follow line until right IR sensor detects an obstacle
    snprintf(lines[line++], MAX_LEN, "vel=0.5, acc=1, edgel=-1, white=1 : ir2 < 0.20");
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0, acc=2, edgel=-1, white=1: time=2");
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
    WaitForEvent(state, 999);
    break;
  case 999:
  default:
    PlaySound("eom.mp3");
    sleep(3);
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
    PlaySound("m2start.mp3");
    sleep(3);
    float heading = (bridge->pose->h)*180.0/M_PI;

    // printf("Heading = %f\n", heading);

    int line = 0;
    // turn 90 degrees to the left
    snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.5, head=%.1f: turn=90", heading+90.0);
    // drive a bit
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.8: dist=0.05");
    // drive while obstacle is in view 
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.8: ir1 > 0.5");
    // drive forward 30 cm
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.8: dist = 0.22");
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
    WaitForEvent(state, 12);
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
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.8: dist = 0.22");
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
    WaitForEvent(state, 14);
    break;
  case 14:
  {
    // turn 90 deg to the right
    SimpleTurn(state, -90.0);
    // go to wait for finished
    state = 15;
    break;
  }
  case 15:
    WaitForEvent(state, 20);
    break;
  case 20:
  {
    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // drive onto line
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.6: xl=20");
    // continue past line
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.6: xl<4, dist=0.1");
    // Move to line
    snprintf(lines[line++], MAX_LEN, "vel=0.1, acc=0.6: dist=0.1");
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
    WaitForEvent(state, 22);
    break;
  case 22:
  {
    // turn left 90 deg
    SimpleTurn(state);
    // go to wait for finished
    state = 23;
    break;
  }
  case 23:
    WaitForEvent(state, 30);
    break;
  case 30:
  {
    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // follow line
    snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=1, edgel=-1, white=1 : dist=0.1");
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

    // go to wait for finished
    state = 31;
    break;
  }
  case 31:
    WaitForEvent(state, 999);
    break;
  case 999:
  default:
    PlaySound("eom.mp3");
    sleep(3);
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
    PlaySound("m3start.mp3");
    sleep(3);
    state = 1;
    break;
  case 1:
  {
    // drive until reaching a line crossing
    Drive2LineX(state);
    //
    // go to wait for finished
    state = 2;
    break;
  }
  case 2:
    WaitForEvent(state, 3);
    break;
  case 3:
  {
    TurnAndDriveUntilLineX(state);
    // go to wait for finished
    state = 4;
    break;
  }
  case 4:
    WaitForEvent(state, 5);
    break;
  case 5:
  {
    // turn 90 deg to the left
    SimpleTurn(state);
    // go to wait for finished
    state = 6;
    break;
  }
  case 6:
    WaitForEvent(state, 7);
    break;
  case 7:
  {
    PrepareGrabber(state);
    // wait for finished setting servo
    state = 8;
  }
  case 8: 
    WaitForEvent(state, 9);
    break;
  case 9:
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < 1.5){
      SaveHeading();
      state = 12;
      PlaySound("orangeball.mp3");
    }
  case 12: // start ball analysis
  {
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < 1.5)
    { // finished drive and turnrate is zero'ish
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
        if (caseCounter == 0){
          state = 25;
          caseCounter++;
        }
        else{
          caseCounter = 0;
          state = 30;
          // tell the operator
          printf("# case=%d found ball\n", state);
          system("espeak \"found ball.\" -ven+f4 -s130 -a5 2>/dev/null &");
          bridge->send("oled 5 found ball");
        }
        cam->ballFound = 1;
      }
      else
      {
        state = 50;
      }
    }
    break;
  case 25:
  {
    FindBall *v = cam->findBalls->returnDataPointer();

    float xm = v->ballPosition.at<float>(0,0);
    float ym = v->ballPosition.at<float>(0,1);
    //float hm = v->ballAngle;

    float heading = (bridge->pose->h)*180.0/M_PI;

    float turnAng = atan2(ym, xm)*(float)180.0/M_PI;
    printf("Turning angle: %.2f\n", turnAng);

    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 
    snprintf(lines[line++], MAX_LEN, "topos=0, vel=0.1, acc=0.5, head=%.1f: turn=%.1f", heading + turnAng, turnAng);
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0: time=1");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d aligning robot with ball\n", state);
    bridge->send("oled 5 aligning...");
    //
    // go to wait for finished
    state = 26;
    break;
  }
  case 26:
    WaitForEvent(state, 12);
    break;
  case 30: // move to ball
  {
    FindBall *v = cam->findBalls->returnDataPointer();

    float xm = v->ballPosition.at<float>(0,0);
    float ym = v->ballPosition.at<float>(0,1);
    float hm = v->ballAngle;

    // stop some distance in front of marker
    float dx = 0.285; // distance to stop in front of marker
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

      float heading = (bridge->pose->h)*180.0/M_PI;
      // stop a few seconds
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
      // open grabber
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=300, vservo=10");
      // turn 
      snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.5, head=%.1f: turn=%.1f", heading + pp4.turnArc1 * 180 / M_PI, pp4.turnArc1 * 180 / M_PI);
      // stop a few seconds
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
      // drive forward
      snprintf(lines[line++], MAX_LEN, "vel=%.3f,acc=%.1f :dist=%.3f", pp4.straightVel, acc, pp4.straightDist);
      // stop a few seconds
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
      // turn 
      snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.5, head=%.1f: turn=%.1f", heading + pp4.turnArc1 * 180 / M_PI + pp4.turnArc2 * 180 / M_PI, pp4.turnArc2 * 180 / M_PI);
      // stop a few seconds
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
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
      state = 42;
    }
    v->lock.unlock();
    
    break;
  }
  case 31: // check if movement is finished
    WaitForEvent(state, 40, 2);
    break;
  case 40:
  {
    GrabBall(state);    
    // go to wait for finished
    state = 41;
    break;
  }
  case 41:
    WaitForEvent(state, 42);
    break;
  case 42:
  {
    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
    // back up
    snprintf(lines[line++], MAX_LEN, "vel=-0.2, acc=1 : dist=0.3");
    // stop and create an event when arrived at this point
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 4 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet\n", state);
    //
    // go to wait for finished
    state = 43;
    break;
  }
  case 43:
    WaitForEvent(state, 44);
    break;
  case 44: // start ball analysis
  {
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < 1.5)
    { // finished drive and turnrate is zero'ish
      state = 45;
      // wait further 30ms - about one camera frame at 30 FPS
      usleep(35000);
      // start ball analysis
      printf("# started new ball analysis\n");
      cam->doFindBall = true;
    }
    break;
  }
  case 45:
    if (not cam->doFindBall) // ball processing finished
    {
      if (cam->ballFound == 0)
      { // found a single ball
        state = 12;
        PlaySound("bruh.mp3");
      }
      else
      { // ball has been picked up
        state = 50;
        PlaySound("yeah.mp3");
      }
    }
    break;
  case 50:
  {
    // method to find the line
    FindLineAfterBall(state);
    // go to wait for finished
    state = 51;
    break;
  }
  case 51:
    WaitForEvent(state, 52);
    break;
  case 52:
    Drive2LineX(state);
    state = 53;
    break;
  case 53:
    WaitForEvent(state, 54);
    break;
  case 54:
  {
    TurnAndDriveUntilLineX(state);
    // go to wait for finished
    state = 55;
    break;
  }
  case 55:
    WaitForEvent(state, 56);
    break;
  case 56:
  {
    // turn 90 deg to the right
    SimpleTurn(state, -90.0);
    // go to wait for finished
    state = 57;
    break;
  }
  case 57:
    WaitForEvent(state, 58);
    break;
  case 58:
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < 1.5){
      SaveHeading();
      // we want robot to go opposite of its current heading
      heading_ref += 180.0;
      heading_ref = heading_ref/180.0*M_PI;
      heading_ref = atan2(sin(heading_ref), cos(heading_ref))*180.0/M_PI;
      state = 60;
    }
    break;
  case 60:
  {
    float heading = (bridge->pose->h)*180.0/M_PI;

    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 90 degrees to the right
    snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.5, head=%.1f: turn=30", heading+30.0);
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
    // drive forward slowly 0.3 m
    snprintf(lines[line++], MAX_LEN, "topos=0.3, vel=0.3, acc=0.6");
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
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
    //
    // go to wait for finished
    state = 61;
    break;
  }
  case 61:
    WaitForEvent(state, 70);
    break;
  case 70:
  {
    DeliverBall(state);
    
    // go to wait for finished
    state = 71;
    break;
  }
  case 71:
    WaitForEvent(state, 80);
    break;
  case 80:
  {
    // method to find the line
    FindLineAfterBall(state, 60.0);
    // go to wait for finished
    state = 81;
    break;
  }
  case 81:
    WaitForEvent(state, 82);
    break;
  case 82:
    Drive2LineX(state);
    state = 83;
    break;
  case 83:
    WaitForEvent(state, 999);
    break;
  case 90: // this case is for testing mission 3
  {
    int line = 0;
    // lower arm
    snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=300, vservo=10");
    // wait 2 seconds
    snprintf(lines[line++], MAX_LEN, ": time = 2");
    // lower arm slowly
    snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=480, vservo=1");
    // wait 3 seconds
    snprintf(lines[line++], MAX_LEN, ": time = 3");
    // drive backwards
    snprintf(lines[line++], MAX_LEN, "vel=-0.2: dist=0.7");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d lowering arm\n", state);
    
    // go to wait for finished
    state = 91;
    break;
  }
  case 91:
    WaitForEvent(state, 999);
    break;
  case 999: // end
  default:
    PlaySound("eom.mp3");
    sleep(3);
    printf("mission 3 ended \n");
    bridge->send("oled 5 \"mission 3 ended.\"");
    finished = true;
    break;
  }
  return finished;
}

/**
 * Run mission
 * Aruco
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
    PlaySound("m4start.mp3");
    sleep(3);
    // tell the operatior what to do
    printf("# started mission 4.\n");
    caseCounter = 0;
    state++;
    break;
  case 1:
  {
    TurnAndDriveUntilLineX(state);
    // go to wait for finished
    state = 2;
    break;
  }
  case 2:
    WaitForEvent(state, 5);
    break;
  case 5:
    // wait for finished driving 
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < 1.5)
    { // finished first drive and turnrate is zero'ish
      state = 6;
      // wait further 30ms - about one camera frame at 30 FPS
      usleep(35000);
      // start aruco analysis 
      PlaySound("aruco.mp3");
      sleep(3);
      printf("# started new ArUco analysis\n");
      cam->doArUcoAnalysis = true;
    }
    break;
  case 6:
  {
    if (not cam->doArUcoAnalysis)
    { // aruco processing finished
      if(cam->markerId == -1)
      {
        printf("No single marker found\n");
        if(caseCounter++ > 3){
          printf("End of mission.\n");
          state = 90;
        }
        else
        {
          usleep(30000);
          state = 5;
        }
      }
      else
      {
        printf("Marker ID: %d\n", cam->markerId);
        state = 7;
        caseCounter = 0;
      }
    }
    break;
  }
  case 7:
  {
    // turn 90 deg to the left
    SimpleTurn(state);
    // go to wait for finished
    state = 8;
    break;
  }
  case 8:
    WaitForEvent(state, 9);
    break;
  case 9:
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < 1.5){
      SaveHeading();
      state = 12;

      if (cam->markerId == 1){
        PlaySound("aur1.mp3");
        sleep(3);
        PlaySound("/blueball.mp3");
        sleep(2);
      }else{
        PlaySound("aur0.mp3");
        sleep(3);
        PlaySound("orangeball.mp3");
        sleep(2);
      }
    }
    break;
  case 12: // start ball analysis
  {
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < 1.5)
    { // finished drive and turnrate is zero'ish
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
        if (caseCounter == 0){
          state = 25;
          caseCounter++;
        }
        else{
          caseCounter = 0;
          state = 30;
          // tell the operator
          printf("# case=%d found ball\n", state);
          system("espeak \"found ball.\" -ven+f4 -s130 -a5 2>/dev/null &");
          bridge->send("oled 5 found ball");
        }
      }
      else
      {
        if (ballGrabTries > 0) // Making sure Sally have tried to grab a ball before moving on
        {
          printf("I have gone through atleast 1 try.. \n");
          ballGrabTries = 0;
          state = 50;
        } else{
          printf("I have not gone through any tries.. \n");
          ballGrabTries++;
          state = 12;
        }
      }
    }
    break;
  case 25:
  {
    FindBall *v = cam->findBalls->returnDataPointer();

    float xm = v->ballPosition.at<float>(0,0);
    float ym = v->ballPosition.at<float>(0,1);
    //float hm = v->ballAngle;

    float heading = (bridge->pose->h)*180.0/M_PI;

    float turnAng = atan2(ym, xm)*(float)180.0/M_PI;
    printf("Turning angle: %.2f\n", turnAng);

    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 
    snprintf(lines[line++], MAX_LEN, "topos=0, vel=0.3, acc=0.5, head=%.1f: turn=%.1f", heading + turnAng, turnAng);
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0: time=1");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d aligning robot with ball\n", state);
    bridge->send("oled 5 aligning...");
    //
    // go to wait for finished
    state = 26;
    break;
  }
  case 26:
    WaitForEvent(state, 12);
    break;
  case 30: // move to ball
  {
    FindBall *v = cam->findBalls->returnDataPointer();

    float xm = v->ballPosition.at<float>(0,0);
    float ym = v->ballPosition.at<float>(0,1);
    float hm = v->ballAngle;

    // stop some distance in front of marker
    float dx = 0.285; // distance to stop in front of marker
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

      float heading = (bridge->pose->h)*180.0/M_PI;
      // stop a few seconds
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
      // open grabber
      snprintf(lines[line++], MAX_LEN, "servo=2, pservo=300, vservo=10");
      // turn 
      snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.5, head=%.1f: turn=%.1f", heading + pp4.turnArc1 * 180 / M_PI, pp4.turnArc1 * 180 / M_PI);
      // stop a few seconds
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
      // drive forward
      snprintf(lines[line++], MAX_LEN, "vel=%.3f,acc=%.1f :dist=%.3f", pp4.straightVel, acc, pp4.straightDist);
      // stop a few seconds
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
      // turn 
      snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.5, head=%.1f: turn=%.1f", heading + pp4.turnArc1 * 180 / M_PI + pp4.turnArc2 * 180 / M_PI, pp4.turnArc2 * 180 / M_PI);
      // stop a few seconds
      snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=3");
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
      state = 42;
    }
    v->lock.unlock();
    
    break;
  }
  case 31: // check if movement is finished
    WaitForEvent(state, 40, 2);
    break;
  case 40:
  {
    GrabBall(state);    
    // go to wait for finished
    state = 41;
    break;
  }
  case 41:
    WaitForEvent(state, 42);
    break;
  case 42:
  {    
    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
    // back up
    snprintf(lines[line++], MAX_LEN, "vel=-0.2, acc=1 : dist=0.3");
    // stop and create an event when arrived at this point
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 4 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d sent mission snippet\n", state);
    //
    // go to wait for finished
    state = 43;
    break;
  }
  case 43:
    WaitForEvent(state, 44);
    break;
  case 44: // start ball analysis
  {
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < 1.5)
    { // finished drive and turnrate is zero'ish
      state = 45;
      // wait further 30ms - about one camera frame at 30 FPS
      usleep(35000);
      // start ball analysis
      printf("# started new ball analysis\n");
      cam->doFindBall = true;
    }
    break;
  }
  case 45:
    if (not cam->doFindBall) // ball processing finished
    {
      if (cam->ballFound == 0)
      { // found a ball
        state = 12;
        PlaySound("bruh.mp3");
      }
      else
      { // ball has been picked up
        state = 50;
        PlaySound("yeah.mp3");
      }
    }
    break;
  case 50:
  {
    // method to find the line
    FindLineAfterBall(state);
    // go to wait for finished
    state = 51;
    break;
  }
  case 51:
    WaitForEvent(state, 52);
    break;
  case 52:
    Drive2LineX(state);
    state = 53;
    break;
  case 53:
    WaitForEvent(state, 54);
    break;
  case 54:
  {
    TurnAndDriveUntilLineX(state);
    // go to wait for finished
    state = 55;
    break;
  }
  case 55:
    WaitForEvent(state, 56);
    break;
  case 56:
  {
    // turn 90 deg to the right
    SimpleTurn(state, -90.0);
    // go to wait for finished
    state = 57;
    break;
  }
  case 57:
    WaitForEvent(state, 58);
    break;
  case 58:
    if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < 1.5){
      SaveHeading();
      // we want robot to go opposite of its current heading
      heading_ref += 180.0;
      heading_ref = heading_ref/180.0*M_PI;
      heading_ref = atan2(sin(heading_ref), cos(heading_ref))*180.0/M_PI;
      state = 60;
    }
    break;
  case 60:
  {
    float heading = (bridge->pose->h)*180.0/M_PI;

    int line = 0;
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 90 degrees to the right
    snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.6, head=%.1f: turn=30", heading+30.0);
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
    // drive forward slowly 0.3 m
    snprintf(lines[line++], MAX_LEN, "topos=0.3, vel=0.3, acc=0.6");
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=2");
    // create event 1
    snprintf(lines[line++], MAX_LEN, "event=1, vel=0");
    // add a line, so that the robot is occupied until next snippet has arrived
    snprintf(lines[line++], MAX_LEN, ": dist=1");
    // send the 6 lines to the REGBOT
    sendAndActivateSnippet(lines, line);
    // make sure event 1 is cleared
    bridge->event->isEventSet(1);
    // tell the operator
    printf("# case=%d driving toward goal\n", state);
    //
    // go to wait for finished
    state = 61;
    break;
  }
  case 61:
    WaitForEvent(state, 70);
    break;
  case 70:
  {
    DeliverBall(state);    
    // go to wait for finished
    state = 71;
    break;
  }
  case 71:
    WaitForEvent(state, 80);
    break;
  case 80:
  {
    // method to find the line
    FindLineAfterBall(state, 60.0);

    // go to wait for finished
    state = 81;
    break;
  }
  case 81:
    WaitForEvent(state, 82);
    break;
  case 82:
    Drive2LineX(state);
    state = 83;
    break;
  case 83:
    WaitForEvent(state, 90);
    break;
  case 90: // lower arm to end mission
  {
    int line = 0;
    // lower arm
    snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=300, vservo=10");
    // wait 2 seconds
    snprintf(lines[line++], MAX_LEN, ": time = 2");
    // lower arm slowly
    snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=480, vservo=1");
    // wait 3 seconds
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
    printf("# case=%d lowering arm\n", state);
    
    // go to wait for finished
    state = 91;
    break;
  }
  case 91:
    WaitForEvent(state, 999);
    break;
  case 999:
  default:
    // end log
    PlaySound("eom.mp3");
    sleep(2);
    PlaySound("gb.mp3");
    sleep(2);
    printf("mission 4 ended \n");
    bridge->send("oled 5 \"mission 4 ended.\"");
    finished = true;
    break;
  }
  return finished;
}

/*
* //////////////////////////////////////////
* //////////////////////////////////////////
* /////////// MISSIONS HELPERS /////////////
* //////////////////////////////////////////
* //////////////////////////////////////////
*/
void UMission::WaitForEvent(int &state, int next_state, int event){
  // wait for event
  if (bridge->event->isEventSet(event))
  { // finished first drive
    state = next_state;
  }
}

void UMission::SimpleTurn(int state, float turn){
    float heading = (bridge->pose->h)*180.0/M_PI;

    int line = 0;
    // raise arm
    snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-300, vservo=10");
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 90 degrees to the left
    snprintf(lines[line++], MAX_LEN, "topos=0, vel=0.3, acc=0.5, head=%.1f: turn=%.1f", heading+turn, turn);
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // align
    //snprintf(lines[line++], MAX_LEN, "vel=0, edgel=-1, white=1 : time=1");
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
}


void UMission::TurnAndDriveUntilLineX(int state, float turn){
  float heading = (bridge->pose->h)*180.0/M_PI;

    int line = 0;
    // raise arm
    snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-300, vservo=10");
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // turn 90 degrees to the left
    snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.5, head=%.1f: turn=%.1f", heading+turn, turn);
    // stop a few seconds
    snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
    // drive a bit forward
    snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=1, edgel=-1, white=1 : dist=0.2");
    // drive until line crossing
    snprintf(lines[line++], MAX_LEN, "edgel=-1, white=1 : xl=20");
    // drive until line is crossed fully
    snprintf(lines[line++], MAX_LEN, "vel=0.1: xl<4, dist=0.1");
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
}

void UMission::FindLineAfterBall(int& state, int direc){
  float heading = (bridge->pose->h)*180.0/M_PI;

  float turnang = heading_ref-90-heading;

  int line = 0;
  // raise arm
  snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-300, vservo=10");
  // wait 2 seconds
  snprintf(lines[line++], MAX_LEN, ": time = 1");
  // drive forward a bit
  snprintf(lines[line++], MAX_LEN, "vel=0.2: dist=0.2");
  // stop a few seconds
  snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
  // turn towards line
  //snprintf(lines[line++], MAX_LEN, "vel=0.3, acc=0.5, head=%.1f: turn=%.1f", heading+ang, ang);
  snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=0.5, head=%.1f: turn=%.1f", heading_ref-90, turnang);
  // stop a few seconds
  snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
  // drive until line crossing
  snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=1, edgel=-1, white=1: xl=20");
  // drive until after crossing
  snprintf(lines[line++], MAX_LEN, "vel=0.1: dist=0.1");
  // stop a few seconds
  snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=1");
  // turn to saved heading
  snprintf(lines[line++], MAX_LEN, "topos=0, vel=0.3, acc=0.5, head=%.1f: time=2", heading_ref);
  // wait 2 seconds
  snprintf(lines[line++], MAX_LEN, ": time = 0.5");
  // create event 1
  snprintf(lines[line++], MAX_LEN, "event=1, vel=0.0");
  // add a line, so that the robot is occupied until next snippet has arrived
  snprintf(lines[line++], MAX_LEN, ": dist=1");
  // send the 6 lines to the REGBOT
  sendAndActivateSnippet(lines, line);

  // make sure event 1 is cleared
  bridge->event->isEventSet(1);
  // tell the operator
  printf("# case=%d sent mission snippet\n", state);
}

void UMission::Drive2LineX(int state){
  int line = 0;
  // raise arm
  snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-300, vservo=10");
  // drive a bit forward slowly
  snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=3, edgel=-1, white=1: dist=0.4");
  // speed up until line
  snprintf(lines[line++], MAX_LEN, "vel=0.5, edgel=-1, white=1: xl=20");
  // stop a few seconds
  snprintf(lines[line++], MAX_LEN, "vel=0.0, acc=10 : time=1.5");
  // back up
  //snprintf(lines[line++], MAX_LEN, "vel=-0.3: dist=0.1");
  // stop a few seconds
  snprintf(lines[line++], MAX_LEN, "vel=0.0 : time=0.5");
  // drive slowly to line
  //snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=1, edgel=-1, white=1: dist=0.15");
  // drive slowly to line
  //snprintf(lines[line++], MAX_LEN, "vel=0.2, acc=1, edgel=-1, white=1: xl=20");
  // continue
  //snprintf(lines[line++], MAX_LEN, "vel=0.2, edgel=-1, white=1: dist=0.1");
  // drive until after crossing
  //snprintf(lines[line++], MAX_LEN, "vel=0.1, acc=1, edgel=-1, white=1: dist=0.05");
  // stop a few seconds
  //snprintf(lines[line++], MAX_LEN, "vel=0.0, acc=2 : time=2");
  // create event 1
  snprintf(lines[line++], MAX_LEN, "event=1, vel=0.0");
  // add a line, so that the robot is occupied until next snippet has arrived
  snprintf(lines[line++], MAX_LEN, ": dist=1");
  // send the 6 lines to the REGBOT
  sendAndActivateSnippet(lines, line);
  // make sure event 1 is cleared
  bridge->event->isEventSet(1);
  // tell the operator
  printf("# case=%d sent mission snippet\n", state);
}

void UMission::PrepareGrabber(int state){
  int line = 0;
  // raise arm
  snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-300, vservo=10");
  // open grabber
  snprintf(lines[line++], MAX_LEN, "servo=2, pservo=300, vservo=10");
  // wait 4 seconds
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
  printf("# case=%d sent mission snippet\n", state);
}

void UMission::GrabBall(int state){
  int line = 0;
  // lower arm
  snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=300, vservo=10");
  // wait 2 seconds
  snprintf(lines[line++], MAX_LEN, ": time = 2");
  // lower arm
  snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=490, vservo=1");
  // wait 2 seconds
  snprintf(lines[line++], MAX_LEN, ": time = 2");
  // close grabber
  snprintf(lines[line++], MAX_LEN, "servo=2, pservo=-200, vservo=10");
  // wait 2 seconds
  snprintf(lines[line++], MAX_LEN, ": time = 2");
  // raise arm
  snprintf(lines[line++], MAX_LEN, "servo=3, pservo=-300, vservo=10");
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
  printf("# case=%d sent mission snippet\n", state);
}

void UMission::DeliverBall(int state){
  int line = 0;
  // lower arm
  snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=250, vservo=10");
  // wait 2 seconds
  snprintf(lines[line++], MAX_LEN, ": time = 2");
  // lower arm slowly
  snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=355, vservo=1");
  // wait 2 seconds
  snprintf(lines[line++], MAX_LEN, ": time = 2");
  // open grabber
  snprintf(lines[line++], MAX_LEN, "servo=2, pservo=300, vservo=10");
  // wait 4 seconds
  snprintf(lines[line++], MAX_LEN, ": time = 2");
  // raise arm
  snprintf(lines[line++], MAX_LEN, "vel=0, servo=3, pservo=-300, vservo=10");
  // wait 2 seconds
  snprintf(lines[line++], MAX_LEN, ": time = 2");
  // back up
  snprintf(lines[line++], MAX_LEN, "vel=-0.3: dist=0.5");
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
}

void UMission::SaveHeading(){
  heading_ref = (bridge->pose->h);
  heading_ref = atan2(sin(heading_ref), cos(heading_ref))*180.0/M_PI;
  cout << "Heading reference: " << heading_ref << endl;
}

void UMission::PlaySound(const char* file){
  // path to folder with sounds
  const char* path = "../sounds/";

  // create new string
	char buffer[256];
	strncpy_s(buffer, path, sizeof(buffer));
	strncat_s(buffer, file, sizeof(buffer));

  // edit original input
	file = (char*)&buffer;

  play.setFile(file);
  play.setVolume(100); // % (0..100)
  play.start();
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
