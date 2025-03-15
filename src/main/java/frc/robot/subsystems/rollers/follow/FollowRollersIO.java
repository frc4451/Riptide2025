package frc.robot.subsystems.rollers.follow;

import org.littletonrobotics.junction.AutoLog;

public interface FollowRollersIO {
  @AutoLog
  public static class FollowRollersMagicIOInputs {
    public boolean connected = false;

    public double leaderPositionRotations = 0.0;
    public double leaderVelocityRotationsPerSec = 0.0;

    public double leaderAppliedVoltage = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderTemperatureCelsius = 0.0;

    public double followerPositionRotations = 0.0;
    public double followerVelocityRotationsPerSec = 0.0;

    public double followerAppliedVoltage = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerTemperatureCelsius = 0.0;

    public double positionGoalRotations = 0.0;
    public double positionSetpointRotations = 0.0;
    public double velocitySetpointRotationsPerSec = 0.0;
  }

  public default void updateInputs(FollowRollersMagicIOInputs inputs) {}

  /** Run rollers at set voltage */
  public default void runVolts(double volts) {}

  /** Set goal position of rollers to position */
  public default void setGoal(double positionRotations) {}

  /** Reset encoders to position */
  public default void resetPosition(double positionRotations) {}

  /** Stop rollers */
  public default void stop() {}
}
