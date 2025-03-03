package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.superstructure.can_range.CanRangeIO;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import org.littletonrobotics.junction.Logger;

public class ElevatorV2 {
  protected final String name = "ElevatorV2";

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final Alert disconnected;

  public static final double atGoalToleranceInches = 1;

  // private final CanRange heightSensor;

  // private final CustomElevatorFF feedforward;

  // private final PIDController positionController;

  private final double inchesPerRad;
  private final ElevatorConstraints elevatorConstraints;

  private SuperStructureModes currentMode;

  private boolean isClosedLoop;

  public ElevatorV2(
      String name,
      ElevatorIO io,
      CanRangeIO heightSensorIO,
      TrapezoidProfile.Constraints trapezoidConstraints,
      double inchesPerRad,
      ElevatorConstraints elevatorConstraints,
      CustomElevatorFF feedforward,
      double kP,
      double kD) {
    this.io = io;

    this.elevatorConstraints = elevatorConstraints;
    this.inchesPerRad = inchesPerRad;

    this.isClosedLoop = true;
    this.disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    this.disconnected.set(!inputs.connected);

    if (DriverStation.isDisabled()) {
      io.stop();
    } else {

    }

    Logger.recordOutput(
        name + "/Profile/SetpointInches",
        new LoggedTrapezoidState(inputs.positionInches, inputs.velocityInchesPerSec));

    Logger.recordOutput(
        name + "/Profile/GoalInches", new LoggedTrapezoidState(inputs.positionGoalInches, 0.0));
  }

  public void runVolts(double inputsVolts) {
    io.runVolts(inputsVolts);
  }

  public void stop() {
    io.stop();
  }

  public double getHeightInches() {
    return getEncoderHeightInches();
  }

  public double getEncoderHeightInches() {
    return inputs.leaderPositionRad * this.inchesPerRad;
  }

  public double getGoalHeightInches() {
    return inputs.positionGoalInches;
  }

  public void setHeightInches(double positionInches) {
    this.io.setPosition(positionInches);
  }

  public void setGoalHeightInches(double positionInches) {
    this.io.setPositionGoal(
        MathUtil.clamp(
            positionInches,
            elevatorConstraints.minHeightInches(),
            elevatorConstraints.maxHeightInches()));
    // double clampedPositionIn =
    //     MathUtil.clamp(
    //         positionInches,
    //         elevatorConstraints.minHeightInches(),
    //         elevatorConstraints.maxHeightInches());
    // goal = new TrapezoidProfile.State(clampedPositionIn, 0.0);
  }
}
