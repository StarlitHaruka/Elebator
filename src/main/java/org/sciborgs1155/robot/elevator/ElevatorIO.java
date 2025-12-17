package org.sciborgs1155.robot.elevator;

public interface ElevatorIO {

  public void setVoltage(double v);

  public double getPos();

  public double getVel();

  public void resetPos();
}
