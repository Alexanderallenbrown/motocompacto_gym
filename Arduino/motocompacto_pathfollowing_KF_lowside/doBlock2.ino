void doBlock2() { //check over these, hole
  t1 = s1&&(!goButtonState); //s1 latch: motor not active
  t2 = s1&&(goButtonState); //s1->s2: motor active
  t3 = s2&&(!su_sig); //s2 latch:kickstand DOWN, < 0.1s
  t4 = s2&&(su_sig); //s2->s3: kickdown UP after timer reaches 0.1s
  t5 = s3&&(goButtonState); //s3 latch: motor active w kickstand stowe
  t6 = s3&&(!goButtonState); //s3 -> s4: motor not active, kickstand coming down
  t7 = s4&&(!sd_sig);//s4 latch: motor active, swinging down
  t8 = s4&&(sd_sig);//s4->s1: roll angle reaches 5 degrees or swing down timer reaches 2 secs (and? or? hm...)
  }
