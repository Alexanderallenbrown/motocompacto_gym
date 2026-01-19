void doBlock1() {
  //read current time in microseconds, calculate an overall loop time.
  t = micros() / 1.0e6;
  dt - t - told;
  sendData();//sends data over websocket to app.js.

  //read the steer angle from the MG90S servo
  //steerFB = servocounts2rad * 180 / PI * (analogRead(2) - servoZeroCounts);

  //read speed based on CAN bus
  canSpeed(U);
  speedFieldValue = String(U) + " mph";
  //  speedFieldValue = String(eCountsSaved);
  oldCounts = eCountsSaved;

  //  readStartButton(); //gives us our motorGo signal
  readIMU(); //reads our IMU
  readGPS(); //reads our GPS
  getPathError();
  //swing up timer logic: if not in s2, elapsed timer hasn't started yet, start timer on
  if (!s2) {
    su_elapsed = 0;
    su_start = millis();
  }
  else {
    su_elapsed = millis() - su_start;
    su_sig = (su_elapsed >= 3000);
  }

  if (!s4) {
    sd_elapsed = 0;
    sd_start = millis();
  }
  else {
    sd_elapsed = millis() - sd_start;
    sd_sig = (sd_elapsed >= 100);
    roll_sig = (rollFinal >= -5.0);
  }
  told = t;
}
