package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PoseUtils;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveParallellToPoseCommand extends Command {
  private final ProfiledPIDController perpendicularController =
      DriveCommandConstants.makeTranslationController();

  private final ProfiledPIDController angleController = DriveCommandConstants.makeAngleController();

  private final Drive drive;
  private final boolean useConstrainedPose;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final DoubleSupplier parallelInput;

  private double parallelError = 0;

  public DriveParallellToPoseCommand(
      Drive drive,
      boolean useConstrainedPose,
      Supplier<Pose2d> targetPoseSupplier,
      DoubleSupplier parallelInput) {
    addRequirements(drive);

    this.drive = drive;
    this.useConstrainedPose = useConstrainedPose;
    this.targetPoseSupplier = targetPoseSupplier;
    this.parallelInput = parallelInput;
  }

  public DriveParallellToPoseCommand withJoystickRumble(Command rumbleCommand) {
    atSetpoint(AlignRoutines.distanceShootTolerance, AlignRoutines.rotationShootTolerance)
        .onTrue(Commands.deferredProxy(() -> rumbleCommand));

    return this;
  }

  public DriveParallellToPoseCommand withJoystickRumble(
      DoubleSupplier rumbleDistance, Command rumbleCommand) {
    atSetpoint(rumbleDistance).onTrue(Commands.deferredProxy(() -> rumbleCommand));

    return this;
  }

  @Override
  public void execute() {
    Pose2d robotPose = useConstrainedPose ? drive.getConstrainedPose() : drive.getGlobalPose();
    Pose2d targetPose = targetPoseSupplier.get();
    Logger.recordOutput("Commands/" + getName() + "/targetPose", targetPose);

    Translation2d translationalError =
        robotPose.getTranslation().minus(targetPose.getTranslation());

    Rotation2d desiredTheta = targetPose.getRotation().plus(Rotation2d.kPi);
    double thetaError = robotPose.getRotation().minus(desiredTheta).getRadians();

    double perpendicularError = PoseUtils.getPerpendicularError(robotPose, targetPose);
    double perpendicularSpeed = perpendicularController.calculate(-perpendicularError, 0);
    perpendicularSpeed = !perpendicularController.atSetpoint() ? perpendicularSpeed : 0;

    parallelError = PoseUtils.getParallelError(robotPose, targetPose);

    double angularSpeed = angleController.calculate(thetaError, 0);
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            -perpendicularSpeed * translationalError.getAngle().getCos(),
            FieldUtils.getFlipped()
                * parallelInput.getAsDouble()
                * drive.getMaxLinearSpeedMetersPerSec(),
            angularSpeed,
            drive.getRotation());

    drive.runVelocity(speeds);

    Logger.recordOutput("Commands/" + getName() + "/ThetaError", thetaError);
    Logger.recordOutput("Commands/" + getName() + "/PerpendicularError", perpendicularError);
    Logger.recordOutput("Commands/" + getName() + "/ParallelError", parallelError);
    Logger.recordOutput(
        "Commands/" + getName() + "/PerpendicularAtSetpoint", perpendicularController.atSetpoint());
    Logger.recordOutput("Commands/" + getName() + "/AngleAtSetpoint", angleController.atSetpoint());
  }

  @Override
  public void end(boolean interrupt) {
    perpendicularController.reset(0);
    angleController.reset(0);
  }

  public Trigger atSetpoint(double distanceTolerance, double rotationTolerance) {
    return new Trigger(
        () ->
            Math.abs(perpendicularController.getPositionError()) < distanceTolerance
                && Math.abs(angleController.getPositionError()) < rotationTolerance);
  }

  public Trigger atSetpoint(DoubleSupplier distance) {
    return new Trigger(
        () ->
            perpendicularController.atSetpoint()
                && angleController.atSetpoint()
                && Math.abs(parallelError) < distance.getAsDouble());
  }
}
