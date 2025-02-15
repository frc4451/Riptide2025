package frc.robot.subsystems.rollers.single;

import org.littletonrobotics.junction.AutoLog;

public interface SingleRollerIO {
  @AutoLog
  public static class SingleRollerIOInputs {
    public boolean connected = false;

    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;

    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public default void updateInputs(SingleRollerIOInputs inputs) {}

  /** Run roller at set voltage */
  public default void runVolts(double volts) {}

  /** Run roller at set velocity */
  public default void runVelocity(double velocityRadPerSec) {}

  /** Run roller to set position */
  public default void runPosition(double positionRad) {}

  /** Reset encoder to position */
  public default void resetPosition(double positionRad) {}

  /** Stop roller */
  public default void stop() {}
}
