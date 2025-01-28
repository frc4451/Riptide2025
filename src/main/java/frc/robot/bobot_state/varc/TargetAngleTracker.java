package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;

public abstract class TargetAngleTracker {
  public abstract void update();

  public abstract Optional<Rotation2d> getRotationTarget();
}
