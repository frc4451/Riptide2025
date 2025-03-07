package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class QuestCalibration {
  // -- Calculate Quest Offset (copied from
  // https://github.com/FRC5010/Reefscape2025/blob/main/TigerShark2025/src/main/java/org/frc5010/common/sensors/camera/QuestNav.java#L65) --

  private Translation2d calculatedOffsetToRobot = new Translation2d();
  private double calculateOffsetCount = 1;

  private Translation2d calculateOffsetToRobot(Pose2d robotPose) {
    Rotation2d angle = robotPose.getRotation();
    Translation2d displacement = robotPose.getTranslation();

    double x =
        ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
            / (2 * (1 - angle.getCos()));
    double y =
        ((-angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
            / (2 * (1 - angle.getCos()));

    return new Translation2d(x, y);
  }

  public Command determineOffsetToRobotCenter(
      Drive drive, Supplier<Pose2d> robotPose, Supplier<Pose2d> questPoseSupplier) {
    return Commands.repeatingSequence(
            Commands.run(
                    () -> {
                      drive.runVelocity(new ChassisSpeeds(0, 0, Math.PI / 10.0));
                    },
                    drive)
                .withTimeout(0.5),
            Commands.runOnce(
                    () -> {
                      // Update current offset
                      Translation2d offset = calculateOffsetToRobot(robotPose.get());

                      calculatedOffsetToRobot =
                          calculatedOffsetToRobot
                              .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                              .plus(offset.div(calculateOffsetCount + 1));
                      calculateOffsetCount++;
                      Logger.recordOutput(
                          "QuestCalibration/CalculatedOffset", calculatedOffsetToRobot);
                    })
                .onlyIf(() -> questPoseSupplier.get().getRotation().getDegrees() > 30))
        .finallyDo(
            () -> {
              // Update current offset
              Translation2d offset = calculateOffsetToRobot(robotPose.get());

              calculatedOffsetToRobot =
                  calculatedOffsetToRobot
                      .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                      .plus(offset.div(calculateOffsetCount + 1));
              calculateOffsetCount++;
              Logger.recordOutput("QuestCalibration/CalculatedOffset", calculatedOffsetToRobot);
            });
  }
}
