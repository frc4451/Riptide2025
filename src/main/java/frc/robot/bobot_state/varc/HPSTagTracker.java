package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldUtils;
import java.util.Optional;

public class HPSTagTracker extends TargetAngleTracker {
  private Optional<Rotation2d> rotationTarget;

  public HPSTagTracker() {
    super();
  }

  public Optional<Rotation2d> getRotationTarget() {
    return rotationTarget;
  }

  public void update() {
    Pose3d robotPose = new Pose3d(BobotState.getGlobalPose());

    // TODO Handle Vision targeting here

    this.rotationTarget =
        Optional.of(FieldUtils.getClosestHPSTag().pose().getRotation().toRotation2d());
  }
}
