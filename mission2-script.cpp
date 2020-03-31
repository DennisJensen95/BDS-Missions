/**
 * Run mission
 * \param state is kept by caller, but is changed here
 *              therefore defined as reference with the '&'.
 *              State will be 0 at first call.
 * \returns true, when finished. */
bool UMission::mission2(int & state)
{
  bool finished = false;
  // First commands to send to robobot in given mission
  // (robot sends event 1 after driving 1 meter)):
  switch (state)
  {
    case 0:
      // tell the operatior what to do
      printf("# started mission 2.\n");
      system("espeak \"looking for ArUco\" -ven+f4 -s130 -a5 2>/dev/null &"); 
      bridge->send("oled 5 looking 4 ArUco");
      state=11;
      break;
    case 11:
      // wait for finished driving first part)
      if (fabsf(bridge->motor->getVelocity()) < 0.001 and bridge->imu->turnrate() < (2*180/M_PI))
      { // finished first drive and turnrate is zero'ish
        state = 12;
        // wait further 30ms - about one camera frame at 30 FPS
        usleep(35000);
        // start aruco analysis 
        printf("# started new ArUco analysis\n");
        cam->arUcos->setNewFlagToFalse();
        cam->doArUcoAnalysis = true;
      }
      break;
    case 12:
      if (not cam->doArUcoAnalysis)
      { // aruco processing finished
        if (cam->arUcos->getMarkerCount(true) > 0)
        { // found a marker - go to marker (any marker)
          state = 30;
          // tell the operator
          printf("# case=%d found marker\n", state);
          system("espeak \"found marker.\" -ven+f4 -s130 -a5 2>/dev/null &"); 
          bridge->send("oled 5 found marker");
        }
        else
        { // turn a bit (more)
          state = 20;
        }
      }
      break;
    case 20: 
      { // turn a bit and then look for a marker again
        int line = 0;
        snprintf(lines[line++], MAX_LEN, "vel=0.25, tr=0.15: turn=10,time=10");
        snprintf(lines[line++], MAX_LEN, "vel=0,event=2:dist=1");
        sendAndActivateSnippet(lines, line);
        // make sure event 2 is cleared
        bridge->event->isEventSet(2);
        // tell the operator
        printf("# case=%d sent mission turn a bit\n", state);
        system("espeak \"turn.\" -ven+f4 -s130 -a5 2>/dev/null &"); 
        bridge->send("oled 5 code turn a bit");
        state = 21;
        break;
      }
    case 21: // wait until manoeuvre has finished
      if (bridge->event->isEventSet(2))
      {// repeat looking (until all 360 degrees are tested)
        if (featureCnt < 36)
          state = 11;
        else
          state = 999;
        featureCnt++;
      }
      break;
    case 30:
      { // found marker
        // if stop marker, then exit
        ArUcoVal * v = cam->arUcos->getID(6);
        if (v != NULL and v->isNew)
        { // sign to stop
          state = 999;
          break;
        }
        // use the first (assumed only one)
        v = cam->arUcos->getFirstNew();
        v->lock.lock();
        // marker position in robot coordinates
        float xm = v->markerPosition.at<float>(0,0);
        float ym = v->markerPosition.at<float>(0,1);
        float hm = v->markerAngle;
        // stop some distance in front of marker
        float dx = 0.3; // distance to stop in front of marker
        float dy = 0.0; // distance to the left of marker
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
          printf("# Sent mission snippet to marker (%d lines)\n", line);
          //system("espeak \"code snippet to marker.\" -ven+f4 -s130 -a20 2>/dev/null &"); 
          bridge->send("oled 5 code to marker");
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
      }
      break;
    case 31:
      // wait for event 2 (send when finished driving)
      if (bridge->event->isEventSet(2))
      { // look for next marker
        state = 11;
        // no, stop
        state = 999;
      }
      break;
    case 999:
    default:
      printf("mission 1 ended \n");
      bridge->send("oled 5 \"mission 1 ended.\"");
      finished = true;
      play.stopPlaying();
      break;
  }
  // printf("# mission1 return (state=%d, finished=%d, )\n", state, finished);
  return finished;
}