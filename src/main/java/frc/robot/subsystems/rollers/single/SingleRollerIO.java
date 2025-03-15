package frc.robot.subsystems.rollers.single;

import org.littletonrobotics.junction.AutoLog;

public interface SingleRollerIO {
  @AutoLog
  public static class SingleRollerIOInputs {
    public boolean connected = false;

    public double positionRotations = 0.0;
    public double velocityRotationsPerSec = 0.0;

    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;

    public double positionGoalRotations = 0.0;
    public double positionSetpointRotations = 0.0;
    public double velocitySetpointRotationsPerSec = 0.0;
  }

  public default void updateInputs(SingleRollerIOInputs inputs) {}

  /** Run roller at set voltage */
  public default void runVolts(double volts) {}

  /** Set goal position of roller to position */
  public default void setGoal(double positionRotations) {}

  /** Reset encoder to position */
  public default void resetPosition(double positionRotations) {}

  /** Stop roller */
  public default void stop() {}
}
