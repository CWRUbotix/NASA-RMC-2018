package com.cwrubotix.glennifer.motor_dispatch;

/* this class contains variables and functions used to control
 * the position of the Pod 
 */
public class Pod {
  private int posistion = 0;    // desired position of the pod (in degree)
  private int turnSpeed = 0;    // pod turning speed
  private float timeout = 0;

  /* pod constructor */
  public Pod (int p, int s, int t) {
    this.position = p;
    this.turnSpeed = s;
    this.timeout = t;
  }

  public void setPosition(int p) {
    position = p;
  }

  public void setSpeed(int s) {
    turnSpeed = s;
  }

  public void setTimeout(int t) {
    timeout = t;
  }
}
