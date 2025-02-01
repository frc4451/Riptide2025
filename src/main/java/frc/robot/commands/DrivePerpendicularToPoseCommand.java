package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DrivePerpendicularToPoseCommand extends Command {
  private final PIDController parallelController =
      DriveCommandConstants.makeTranslationController();

  private final ProfiledPIDController angleController = DriveCommandConstants.makeAngleController();

  private final Drive drive;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final Supplier<Double> perpendicularInput;

  public DrivePerpendicularToPoseCommand(
      Drive drive, Supplier<Pose2d> maybeTargetPose, Supplier<Double> perpendicularInput) {
    addRequirements(drive);

    this.drive = drive;
    this.targetPoseSupplier = maybeTargetPose;
    this.perpendicularInput = perpendicularInput;
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    Pose2d targetPose = targetPoseSupplier.get();

    Rotation2d desiredTheta = targetPose.getRotation().plus(Rotation2d.kPi);

    // https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
    Translation2d robotToTarget = robotPose.minus(targetPose).getTranslation();
    Rotation2d angleBetween = robotToTarget.getAngle();
    double parallelError = -robotToTarget.getNorm() * angleBetween.getSin();
    Logger.recordOutput("Commands/" + getName() + "/parallelError", parallelError);

    Rotation2d thetaError = robotPose.getRotation().minus(desiredTheta);
    Logger.recordOutput("Commands/" + getName() + "/thetaError", thetaError);

    double parallelSpeed = parallelController.calculate(parallelError);

    double angularSpeed = angleController.calculate(thetaError.getRadians());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            perpendicularInput.get() * drive.getMaxLinearSpeedMetersPerSec(),
            parallelSpeed,
            angularSpeed,
            desiredTheta);

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
  }
}
