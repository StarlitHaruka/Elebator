package org.sciborgs1155.robot.elevator;

public class NoElevator implements ElevatorIO{

    @Override
    public void setVoltage(double v) {
    }

    @Override
    public double getPos() {
        return 0.0;
    }

    @Override
    public double getVel() {
        return 0.0;
    }

    @Override
    public void resetPos() {}
    
}
