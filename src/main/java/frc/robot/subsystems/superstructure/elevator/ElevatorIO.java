package frc.robot.subsystems.superstructure.elevator;

import frc.robot.subsystems.rollers.follow.FollowRollersIO;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO extends FollowRollersIO {
  @AutoLog
  public static class ElevatorIOInputs extends FollowRollersIOInputs {
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

    public double positionInches = 0.0;
    public double velocityInchesPerSec = 0.0;
    public double positionGoalInches = 0.0;
    public double positionSetpointInches = 0.0;
    public double positionErrorInches = 0.0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the voltage for the elevator.
   *
   * @param volts The voltage to set.
   */
  public default void runVolts(double volts) {}

  /**
   * Sets the position for the elevator.
   *
   * @param meters The position to set in meters.
   */
  public default void setPosition(double positionInches) {}

  /**
   * Sets the position goal for the elevator.
   *
   * @param meters The position goal to set in meters.
   */
  public default void setPositionGoal(double positionInches) {}
}
