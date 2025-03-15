package frc.robot.subsystems.superstructure.elevatorV2;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorV2IO {
  @AutoLog
  public static class ElevatorV2IOInputs {
    public boolean connected = false;

    public double leaderPositionRad = 0.0;
    public double leaderVelocityRadPerSec = 0.0;

    public double leaderAppliedVoltage = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderTemperatureCelsius = 0.0;

    public double followerPositionRad = 0.0;
    public double followerVelocityRadPerSec = 0.0;

    public double followerAppliedVoltage = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerTemperatureCelsius = 0.0;

    public double positionGoalInches = 0.0;
    public double positionSetpointInches = 0.0;
    public double positionErrorInches = 0.0;
  }

  public default void updateInputs(ElevatorV2IOInputs inputs) {}

  /** Run rollers at set voltage */
  public default void runVolts(double volts) {}

  /** Reset encoders to position */
  public default void resetPosition(double positionRad) {}

  /** Stop rollers */
  public default void stop() {}

  /** Sets the position for the elevator. */
  public default void setPosition(double positionRad) {}

  /** Sets the position goal for the elevator. */
  public default void setPositionGoalInches(double positionInches) {}

  public default double getPositionInches() {
    return 0.0;
  }

  public default double getPositionGoalInches() {
    return 0.0;
  }
}
