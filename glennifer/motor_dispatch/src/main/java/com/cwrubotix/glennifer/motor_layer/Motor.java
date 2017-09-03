package com.cwrubotix.glennifer.motor_dispatch;

/* this class responsible for the motor speed */
public class Motor {
  private int rmp = 0;        // motor rpm
  private float dist = 0;     // the desired moving distance 
  private float timeout = 0;  // timeout

  /* class constructor */
  public Motor (int r, float d, float t) {
    this.rmp = r;
    this.dist = d;
    this.timeout = t;
  }

  public void setRpm(int r) {
    this.rmp = r;
  }

  public void setDistance(int d) {
    this.dist = d;
  }

  public void setTimeout (float t) {
    this.timeout =t;
  }
}



