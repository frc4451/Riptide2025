package frc.robot.bobot_state.varc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.BobotState;
import frc.robot.field.FieldConstants.ReefFace;
import frc.robot.field.FieldUtils;
import java.util.Optional;

public class ReefTagTracker extends TargetAngleTracker {
  private Optional<Rotation2d> rotationTarget;

  private Optional<ReefFace> closestReef;

  public ReefTagTracker() {
    super();
  }

  public Optional<Rotation2d> getRotationTarget() {
    return rotationTarget;
  }

  public Optional<ReefFace> getClosestReef() {
    return closestReef;
  }

  public void update() {
    Pose3d robotPose = new Pose3d(BobotState.getGlobalPose());

    // TODO Handle Vision targeting here

    this.rotationTarget =
        Optional.of(
            FieldUtils.getClosestReef()
                .tag()
                .pose()
                .getRotation()
                .toRotation2d()
                .plus(new Rotation2d(Math.PI)));

    this.closestReef = Optional.of(FieldUtils.getClosestReef());

    // this.rotationTarget =
    //     Optional.of(
    //         FieldUtils.getClosestReefAprilTag()
    //             .pose()
    //             .relativeTo(robotPose)
    //             .getTranslation()
    //             .toTranslation2d()
    //             .getAngle()
    //             .plus(BobotState.getGlobalPose().getRotation()));
  }
}
