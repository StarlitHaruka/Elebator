package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
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

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;
import org.sciborgs1155.robot.elevator.ElevatorConstants.Level;

@Logged
public class Elevator extends SubsystemBase {

  private final ElevatorIO hardware;

  private final SysIdRoutine sysIdRoutine;

  public static Elevator create() {
    if (Robot.isReal()) {
      return new Elevator(new RealElevator());
    } else {
      return new Elevator(new SimElevator());
    }
  }

  
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new TrapezoidProfile.Constraints(
              MAX_VELOCITY.in(MetersPerSecond), MAX_ACCEL.in(MetersPerSecondPerSecond)));

  private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV);

  public Elevator(ElevatorIO hardware) {

    this.hardware = hardware;

    setDefaultCommand(retract());

    pid.setTolerance(POSITION_TOLERANCE.in(Meters));
    pid.reset(hardware.getPos());
    pid.setGoal(MIN_EXTENSION.in(Meters));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(2),
                null,
                (state) -> SignalLogger.writeString("elevator state", state.toString())),
            new SysIdRoutine.Mechanism(v -> hardware.setVoltage(v.in(Volts)), null, this));
  }

  private Command goTo(DoubleSupplier h) {
    return run(() -> update(h.getAsDouble())).finallyDo(() -> hardware.setVoltage(0));
  }

  private Command goTo(double h) {
    return goTo(() -> h);
  }

  /**
   * brings elevator to minimum
   *
   * @return command telling elevator to go to MIN_EXTENSION
   */
  private Command retract() {
    return goTo(() -> MIN_EXTENSION.in(Meters));
  }

  /**
   * tells the elevator to go to a specific scoring level
   *
   * @param level
   * @return a command telling the elevator to go to that level
   */
  public Command scoreLevel(Level level) {
    return goTo(level.extension.in(Meters));
  }

  /**
   * copypasted but i assumed allows for manual elevator
   *
   * @param input
   * @return
   */
  public Command manualElevator(InputStream input) {
    return goTo(input
            .deadband(.15, 1)
            .scale(MAX_VELOCITY.in(MetersPerSecond))
            .scale(2)
            .scale(Constants.PERIOD.in(Seconds))
            .rateLimit(MAX_ACCEL.in(MetersPerSecondPerSecond))
            .add(() -> pid.getGoal().position))
        .withName("manual elevator");
  }

  // private Command cleanLevel(Level level) {

  //   /*invalid levels */
  //   if (level == Level.L1 || level == Level.L4) {
  //     return retract();
  //   }
  //   return goTo(level.extension.in(Meters));
  // }

  
  public double pos() {
    return hardware.getPos();
  }

  
  public double vel() {
    return hardware.getVel();
  }

  
  public double getPosSetpoint() {
    return pid.getSetpoint().position;
  }

  
  public double getVelSetpoint() {
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
    hardware.setVoltage(feedf + feedb);
  }

  public void close() throws Exception {
    close();
  }
}
