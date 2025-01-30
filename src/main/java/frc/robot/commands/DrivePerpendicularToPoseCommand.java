package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DrivePerpendicularToPoseCommand extends Command {
  private final PIDController parallelController = new PIDController(5.0, 0.0, 0.0);
  private final PIDController thetaController = new PIDController(5.0, 0.0, 0.0);

  private final Drive drive;
  private final Supplier<Pose2d> maybeTargetPose;
  private final Supplier<Double> perpendicularInput;

  public DrivePerpendicularToPoseCommand(
      Drive drive, Supplier<Pose2d> maybeTargetPose, Supplier<Double> perpendicularInput) {
    addRequirements(drive);

    this.drive = drive;
    this.maybeTargetPose = maybeTargetPose;
    this.perpendicularInput = perpendicularInput;
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose();
    Pose2d targetPose = maybeTargetPose.get();

    Rotation2d desiredTheta = targetPose.getRotation();

    // https://en.wikipedia.org/wiki/Vector_projection#Scalar_projection
    Translation2d robotToTarget = robotPose.minus(targetPose).getTranslation();
    Rotation2d angleBetween = robotToTarget.getAngle();
    double parallelError = robotToTarget.getNorm() * angleBetween.getSin();

    Rotation2d thetaError = robotPose.getRotation().minus(desiredTheta);

    double parallelSpeed = parallelController.calculate(parallelError);

    double angularSpeed = thetaController.calculate(thetaError.getRadians());

    // The error is here, need to fix
    ChassisSpeeds speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            -perpendicularInput.get() * drive.getMaxLinearSpeedMetersPerSec(),
            parallelSpeed,
            angularSpeed,
            desiredTheta);

    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
  }
}
