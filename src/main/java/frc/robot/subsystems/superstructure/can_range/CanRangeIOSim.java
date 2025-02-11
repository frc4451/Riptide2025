package frc.robot.subsystems.superstructure.can_range;

public class CanRangeIOSim implements CanRangeIO {
  public void updateInputs(CanRangeIOInputs inputs) {
    inputs.connected = true;
    inputs.distanceMeters = 1.0;
  }
}
