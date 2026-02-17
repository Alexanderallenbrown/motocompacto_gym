void doBlock4() {
  if (s1) { ///STOP: kickstand leaned down for resting,controller isn't active
    state = 1; //printing purposes
//    writeDriveMotor(0);
//    doUControl(0);
    
    startime = millis(); //experiment run time timer
    writeServos(steerCenterAngle,leanAngle); //kickstand should be leaned over and steer angle should be at 0
    doThrottleManual();
//    doLED(15);
  }

  else if (s2) { //SWING UP STATE: kickstand should pop the bike, controller becomes active
    state = 2; //printing purposes
    goalRoll = 0;
//    writeDriveMotor(driveVal); // full speed is 255; 127 represents speed  around 1 m/s
//    doUControl(goalU);
    openLoopUControl();
    writeServos(steerFinal,upAngle); //kickstand should be up and steer angle should be at 0
    doRollControl();
    rollSliderPosition = 0; //desired roll angle is 0
//    doLED(45);
    //this gives us a "start time" for the experiment. When used in an elapsed time calculation, it tells us how long the "motorGo" variable has been true.
  }
  else if (s3) { //GO STATE
    state = 3; //printing purposes
    Goelapsed = millis() - startime; //start time starting in swing up state
    //writing these two pins high makes the drive motor go full speed forward. Writing them low turns off the drive motor
//    writeDriveMotor(driveVal);
//    doUControl(goalU);
    openLoopUControl();
    writeServos(steerFinal,stowAngle); //kickstand should be stow and not in the way and steer angle should be at 0
    //keep track of how long we've been in the "go" state.
    doPathControl();
    doRollControl();
//    doLED(120);

  }
  else if (s4) { //SWING DOWN STATE
    state = 4;
//    writeDriveMotor(driveVal);
//    doUControl(goalU);
    openLoopUControl();
    //SWING DOWN STATE: kick stand is coming down, and we are leaning to the left in anticipation of stopping.
    //drop the kickstand
    writeServos(steerFinal,leanAngle); //kickstand should be lean over steer angle should be at 0
    //override the joystick for a left lean
    goalRoll = 0;//-12; //desired roll angle is -5 degree,ready to stop 
    //do roll controller still
    doRollControl();
//    doLED(270);
  }
  else {
   writeServos(steerCenterAngle,leanAngle); //kickstand should be lean over steer angle should be at 0
    steer = 0;
    rollSliderPosition = 0; //desired roll angle is 0
    intE = 0;
    writeDriveMotor(0);
  }
}
