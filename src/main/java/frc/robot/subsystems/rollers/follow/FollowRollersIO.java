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

  /** Run roller at set voltage */
  public default void runVolts(double volts) {}

  public default void runVelocity(double velocity) {}

  /** Run roller at set position */
  public default void runPosition(double positionRad) {}

  /** Stop roller */
  public default void stop() {}
}
