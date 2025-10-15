package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.CONVERSION_FACTOR;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.CURRENT_LIMIT;

import org.sciborgs1155.lib.TalonUtils;
import org.sciborgs1155.robot.Ports;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class RealElevator implements ElevatorIO{

    private final TalonFX lead = new TalonFX(Ports.Elevator.FRONT_LEADER);
    private final TalonFX follow = new TalonFX(Ports.Elevator.BACK_FOLLOWER);

    public RealElevator() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        follow.setControl(new Follower(Ports.Elevator.FRONT_LEADER, true));

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = CONVERSION_FACTOR;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT.in(Amps);

        lead.getConfigurator().apply(config);
        follow.getConfigurator().apply(config);

        lead.setPosition(0);
        follow.setPosition(0);

        //unsure what these are for 
        TalonUtils.addMotor(lead);
        TalonUtils.addMotor(follow);

    }

    @Override
    public void setVoltage(double v) {
        lead.setVoltage(v);
    }

    @Override
    public double getPos() {
        return lead.getPosition().getValueAsDouble();
    }

    @Override
    public double getVel() {
        return lead.getVelocity().getValueAsDouble();
    }

    @Override
    public void resetPos() {
        lead.setPosition(0);
    }
    
}
