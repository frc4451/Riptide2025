package frc.robot.subsystems.rollers.single;

import org.littletonrobotics.junction.AutoLog;

public interface SingleRollerIO {
  @AutoLog
  static class SingleRollerIOInputs {
    public boolean connected = false;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  default void updateInputs(SingleRollerIOInputs inputs) {}

  /** Run roller at set voltage */
  default void runVolts(double volts) {}

  /** Stop roller */
  default void stop() {}
}
