package frc.robot.subsystems.superstructure.can_range;

import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;

public class CanRange {
  private static final double thresholdMeters = 0.25;

  protected final String name;
  protected final CanRangeIO io;
  private final CanRangeIOInputsAutoLogged inputs = new CanRangeIOInputsAutoLogged();

  private final Alert disconnected;

  public CanRange(String name, CanRangeIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert("CanRange " + name + " is disconnected", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    Logger.recordOutput(name + "/IsNear", isNear());
    disconnected.set(!inputs.connected);
  }

  public double getDistanceMeters() {
    return inputs.distanceMeters;
  }

  public boolean isNear() {
    return getDistanceMeters() < thresholdMeters;
  }
}
