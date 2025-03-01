package frc.robot.subsystems.superstructure.can_range;

import org.littletonrobotics.junction.AutoLog;

public interface CanRangeIO {
  @AutoLog
  public static class CanRangeIOInputs {
    public boolean connected = false;
    public boolean isDetected = false;
    public double distanceMeters = 0.0;
  }

  public default void updateInputs(CanRangeIOInputs inputs) {}
}
