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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveParallellToPoseCommand extends Command {
  private final ProfiledPIDController xController =
      DriveCommandConstants.makeTranslationController();

  private final ProfiledPIDController angleController = DriveCommandConstants.makeAngleController();

  private final Drive drive;
  private final boolean useConstrainedPose;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final DoubleSupplier parallelInput;

  private double yError = 0;

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
    Pose2d robot = useConstrainedPose ? drive.getConstrainedPose() : drive.getGlobalPose();
    Pose2d target = targetPoseSupplier.get();

    Translation2d translationalError = robot.getTranslation().minus(target.getTranslation());

    Rotation2d desiredTheta = target.getRotation().plus(Rotation2d.kPi);
    double angularError = robot.getRotation().minus(desiredTheta).getRadians();

    double xSpeed = xController.calculate(translationalError.getX(), 0);
    xSpeed = !xController.atSetpoint() ? xSpeed : 0;

    double angularVelocity = angleController.calculate(angularError, 0);
    angularVelocity = !angleController.atSetpoint() ? angularVelocity : 0;

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            FieldUtils.getFlipped()
                * parallelInput.getAsDouble()
                * drive.getMaxLinearSpeedMetersPerSec(),
            angularVelocity,
            drive.getRotation());

    drive.runVelocity(speeds);

    Logger.recordOutput("Commands/" + getName() + "/Robot", robot);
    Logger.recordOutput("Commands/" + getName() + "/Target", target);
    Logger.recordOutput("Commands/" + getName() + "/Error/Translational", translationalError);
    Logger.recordOutput("Commands/" + getName() + "/Error/Angular", angularError);
    Logger.recordOutput("Commands/" + getName() + "/AtGoal/X", xController.atGoal());
    Logger.recordOutput("Commands/" + getName() + "/AtGoal/Angle", angleController.atGoal());
    Logger.recordOutput("Commands/" + getName() + "/Speeds/ChassisSpeeds", speeds);
  }

  @Override
  public void end(boolean interrupt) {
    xController.reset(0);
    angleController.reset(0);
  }

  public Trigger atSetpoint(double distanceTolerance, double rotationTolerance) {
    return new Trigger(
        () ->
            Math.abs(xController.getPositionError()) < distanceTolerance
                && Math.abs(angleController.getPositionError()) < rotationTolerance);
  }

  public Trigger atSetpoint(DoubleSupplier distance) {
    return new Trigger(
        () ->
            xController.atSetpoint()
                && angleController.atSetpoint()
                && Math.abs(yError) < distance.getAsDouble());
  }
}
