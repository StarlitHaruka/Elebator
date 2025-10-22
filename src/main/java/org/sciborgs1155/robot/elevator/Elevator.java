package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_ACCEL;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MAX_VELOCITY;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.MIN_EXTENSION;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.POSITION_TOLERANCE;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kD;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kG;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kI;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kP;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kS;
import static org.sciborgs1155.robot.elevator.ElevatorConstants.kV;

import java.nio.FloatBuffer;
import java.util.function.DoubleSupplier;

import org.sciborgs1155.robot.Robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final ElevatorIO hardware;

    public static Elevator create() {
        if (Robot.isReal()) {
          return new Elevator(new RealElevator());
        } else {
          return new Elevator(new SimElevator());
        }
    
      }
    
    private final ProfiledPIDController pid = new ProfiledPIDController(
        kP, 
        kI, 
        kP, 
        new TrapezoidProfile.Constraints(MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));

    private final ElevatorFeedforward ff = new ElevatorFeedforward(
        kS, 
        kG, 
        kV);

    
    public Elevator(ElevatorIO hardware) {

        this.hardware = hardware;

        setDefaultCommand(retract());

        pid.setTolerance(POSITION_TOLERANCE.in(Meters));
        pid.reset(hardware.getPos());
        pid.setGoal(MIN_EXTENSION.in(Meters));

    }



    private Command goTo(DoubleSupplier h) {
        return run(() -> update(h.getAsDouble())).finallyDo(() -> hardware.setVoltage(0));
    }

    private Command go(double h) {
        return goTo(() -> h);
    }

    private Command retract() {
        return goTo(() -> MIN_EXTENSION.in(Meters));
    }

    @Logged
    private double pos() {
        return hardware.getPos();
    }

    @Logged
    private double vel() {
        return hardware.getVel();
    }

    @Logged
    private double getPosSetpoint() {
        return pid.getSetpoint().position;
    }

    @Logged
    private double getVelSetpoint() {
        return pid.getSetpoint().velocity;
    }


    private void update(double pos) {
        double goal =
            Double.isNaN(pos)
            ? MIN_EXTENSION.in(Meters)
            : MathUtil.clamp(pos, MIN_EXTENSION.in(Meters), MAX_EXTENSION.in(Meters));
        double lastVel = pid.getSetpoint().velocity;
        double feedb = pid.calculate(hardware.getPos(), goal);
        double feedf = ff.calculateWithVelocities(lastVel, pid.getSetpoint().velocity);

    }
}
