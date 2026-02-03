


void doPathControl(){
  goalRoll = (KPrev*ePrev + KdPrev*dEPrev)*180/PI;
  goalRoll = constrain(goalRoll,-15,15);
}

void doRollControl(){
    //filter the goal roll (which is called rollSliderPosition) so that quick changes don't cause the bike to overshoot and tip over.
    goalRoll_filt += dt / desFilt_tau * (goalRoll -  goalRoll_filt);
    //compute error for congtroller
    float e = (goalRoll_filt - rollFinal);

    //only compute the roll integral if enough time has passed since we entered the go state.
    if (Goelapsed > goduration) {//change the name of goduration so it is more descriptive?
      intE += dt * e;
    }
    else {
      intE = 0;
    }
    //constrain error integral to make sure it doesn't get huge. allow it to contribute up to +/- 30 degrees to steer abngle command.
    intE = constrain(intE, -60.0 / Ki, 60.0 / Ki); //with limit integral corrections to +/- 30 degree
    //compute steer angle based on control law
    steer =  Klocus * (Kp * e - Kd * rollRate + Ki * intE); 
    steerFinal = steerCenterAngle + steer;//servo commands use a "center" angle of around 90 degrees, so what we actually send as a command should be 90ish + "steer"
    float steermax = 80.0;
    steerFinal = constrain(steerFinal, steerCenterAngle-steermax, steerCenterAngle+steermax); //allow total steer angles of 50 to -50 degrees, centered at 90ish
}
