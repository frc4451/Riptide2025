package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import java.util.List;
import java.util.Optional;

public record VisionSource(
    String name, Transform3d robotToCamera, Optional<List<AprilTagStruct>> trigTargets) {
  public VisionSource(String name, Transform3d robotToCamera) {
    this(name, robotToCamera, Optional.empty());
  }
}
