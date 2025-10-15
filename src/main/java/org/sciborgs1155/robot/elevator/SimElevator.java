package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.Constants.PERIOD;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.SPROCKET_RADIUS;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.WEIGHT;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimElevator implements ElevatorIO{

    private final ElevatorSim elevator = new ElevatorSim(
        LinearSystemId.createElevatorSystem(
            DCMotor.getKrakenX60(2), WEIGHT.in(Kilograms), SPROCKET_RADIUS.in(Meters), ElevatorConstants.GEARING),
            DCMotor.getKrakenX60(2),
            MIN_EXTENSION.in(Meters),
            MAX_EXTENSION.in(Meters),
            true,
            MIN_EXTENSION.in(Meters));
       
    public SimElevator() {
        elevator.update(PERIOD.in(Seconds));
    }

    @Override
    public void setVoltage(double v) {
        elevator.setInputVoltage(v);
        elevator.update(PERIOD.in(Seconds));
    }

    @Override
    public double getPos() {
        return elevator.getPositionMeters();
    }

    @Override
    public double getVel() {
        return elevator.getVelocityMetersPerSecond();
    }

    @Override
    public void resetPos() {
        elevator.setState(0,0);
        
    }

    
    
}
