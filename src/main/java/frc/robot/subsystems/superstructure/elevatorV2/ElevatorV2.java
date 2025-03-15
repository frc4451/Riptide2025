package frc.robot.subsystems.superstructure.elevatorV2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.rollers.LoggedTrapezoidState;
import frc.robot.subsystems.superstructure.can_range.CanRange;
import frc.robot.subsystems.superstructure.can_range.CanRangeIO;
import frc.robot.subsystems.superstructure.elevator.CustomElevatorFF;
import frc.robot.subsystems.superstructure.elevator.ElevatorConstraints;
import org.littletonrobotics.junction.Logger;

public class ElevatorV2 {
  public static final double isNearToleranceInches = 1;

  private final CanRange heightSensor;

  private final ElevatorV2IO io;
  private final ElevatorV2IOInputsAutoLogged inputs = new ElevatorV2IOInputsAutoLogged();

  private TrapezoidProfile.State goal = new TrapezoidProfile.State();

  private final double inchesPerRad;
  private final ElevatorConstraints elevatorConstraints;

  private final String name;

  private final Alert disconnected;

  public ElevatorV2(
      String name,
      ElevatorV2IO io,
      CanRangeIO heightSensorIO,
      TrapezoidProfile.Constraints trapezoidConstraints,
      double inchesPerRad,
      ElevatorConstraints elevatorConstraints,
      CustomElevatorFF feedforward,
      double kP,
      double kD) {
    this.name = name;
    this.io = io;
    this.heightSensor = new CanRange(name + "/HeightSensor", heightSensorIO);
    this.inchesPerRad = inchesPerRad;
    this.elevatorConstraints = elevatorConstraints;
    Logger.recordOutput(name + "/Feedforward", feedforward);

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    // if (DriverStation.isDisabled()) {
    //   resetController();
    // } else {
    //   runTrapezoidProfile();
    // }
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);

    if (DriverStation.isDisabled()) {
      io.stop();
    }

    runProfile();

    Logger.recordOutput(
        name + "/Profile/GoalInches", new LoggedTrapezoidState(goal.position, goal.velocity));

    Logger.recordOutput(name + "/EncoderHeightInches", getEncoderHeightInches());
    Logger.recordOutput(name + "/HeightInches", getHeightInches());
    Logger.recordOutput(name + "/VelocityInchesPerSec", getVelocityInchesPerSec());
    Logger.recordOutput(name + "/ElevatorHeightInches", io.getPositionInches());
  }

  public double getEncoderHeightInches() {
    return inputs.leaderPositionRad * inchesPerRad;
  }

  /** Defaults to using Encoder Height Inches */
  public double getHeightInches() {
    return getEncoderHeightInches();
  }

  public double getVelocityInchesPerSec() {
    return inputs.leaderVelocityRadPerSec;
  }

  public double getPositionInches() {
    return getEncoderHeightInches();
  }

  public double getPositionGoalInches() {
    return this.io.getPositionGoalInches();
  }

  /**
   * Assuming that this is the Real robot, you should use `Motion Magic` from CTRE If it's the
   * simulated robot, you should use the ProfiledPID controller
   */
  public void runProfile() {
    this.io.setPositionGoalInches(goal.position);
  }

  public void setHeightInches(double positionInches) {
    io.resetPosition(positionInches / inchesPerRad);
  }

  public void setGoalHeightInches(double positionInches) {
    double clampedPositionIn =
        MathUtil.clamp(
            positionInches,
            elevatorConstraints.minHeightInches(),
            elevatorConstraints.maxHeightInches());

    goal = new TrapezoidProfile.State(clampedPositionIn, 0.0);

    io.setPositionGoalInches(goal.position);
  }

  public double getGoalHeightInches() {
    return this.io.getPositionGoalInches();
  }

  // private void resetController() {
  //   setpoint = new TrapezoidProfile.State(getHeightInches(), 0.0);
  // }

  public boolean isNear(double heightInches) {
    return MathUtil.isNear(getHeightInches(), heightInches, isNearToleranceInches);
  }

  public boolean atGoal() {
    return isNear(getGoalHeightInches());
  }

  public void runVolts(double inputsVolts) {
    io.runVolts(inputsVolts);
  }

  public void stop() {
    io.stop();
  }
}
