package frc.robot.subsystems.rollers.follow;

import org.littletonrobotics.junction.AutoLog;

public interface FollowRollersIO {
  @AutoLog
  public static class FollowRollersIOInputs {
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
  }

  public default void updateInputs(FollowRollersIOInputs inputs) {}

  /** Run rollers at set voltage */
  public default void runVolts(double volts) {}

  /** Run rollers at set velocity */
  public default void runVelocity(double velocityRadPerSec) {}

  /** Run rollers to set position */
  public default void runPosition(double positionRad) {}

  /** Reset encoders to position */
  public default void resetPosition(double positionRad) {}

  /** Stop rollers */
  public default void stop() {}
}
