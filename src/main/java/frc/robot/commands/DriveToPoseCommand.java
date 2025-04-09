package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseCommand extends Command {
  private final ProfiledPIDController distanceController =
      DriveCommandConstants.makeTranslationController();
  private final ProfiledPIDController angleController = DriveCommandConstants.makeAngleController();

  private final Drive drive;
  private final boolean useConstrainedPose;
  private final Supplier<Pose2d> targetPoseSupplier;

  public DriveToPoseCommand(
      Drive drive, boolean useConstrainedPose, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);

    this.drive = drive;
    this.useConstrainedPose = useConstrainedPose;
    this.targetPoseSupplier = targetPoseSupplier;
  }

  public DriveToPoseCommand withJoystickRumble(Command rumbleCommand) {
    atSetpoint().onTrue(Commands.deferredProxy(() -> rumbleCommand));

    return this;
  }

  // Heavily inspired by 6328's `AutoAlignController` from 2024
  // https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/drive/controllers/AutoAlignController.java#L135
  @Override
  public void execute() {
    Pose2d robot = useConstrainedPose ? drive.getConstrainedPose() : drive.getGlobalPose();
    Pose2d target = targetPoseSupplier.get();

    Translation2d translationalError = robot.getTranslation().minus(target.getTranslation());
    Rotation2d angularError = robot.getRotation().minus(target.getRotation());

    // Magnitude of translational velocity, meaning that x & y are controlled together
    double translationalSpeed = distanceController.calculate(translationalError.getNorm(), 0);
    translationalSpeed = !distanceController.atGoal() ? translationalSpeed : 0;

    double anglularVelocity = angleController.calculate(angularError.getRadians(), 0);
    anglularVelocity = !angleController.atGoal() ? anglularVelocity : 0;

    Translation2d velocity = new Translation2d(translationalSpeed, translationalError.getAngle());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(), velocity.getY(), anglularVelocity, robot.getRotation());

    drive.runVelocity(speeds);

    Logger.recordOutput("Commands/" + getName() + "/Robot", robot);
    Logger.recordOutput("Commands/" + getName() + "/Target", target);
    Logger.recordOutput("Commands/" + getName() + "/Error/Translational", translationalError);
    Logger.recordOutput("Commands/" + getName() + "/Error/Angular", angularError);
    Logger.recordOutput(
        "Commands/" + getName() + "/AtGoal/Translation", distanceController.atGoal());
    Logger.recordOutput("Commands/" + getName() + "/AtGoal/Angle", angleController.atGoal());
    Logger.recordOutput("Commands/" + getName() + "/Speeds/Translational", translationalSpeed);
    Logger.recordOutput("Commands/" + getName() + "/Speeds/ChassisSpeeds", speeds);
  }

  @Override
  public void end(boolean interrupt) {
    drive.runVelocity(new ChassisSpeeds());
    distanceController.reset(0);
    angleController.reset(0);
  }

  public Trigger atSetpoint() {
    return new Trigger(() -> distanceController.atSetpoint() && angleController.atSetpoint());
  }

  public Trigger atSetpoint(double distanceTolerance, double rotationTolerance) {
    return new Trigger(
        () ->
            Math.abs(distanceController.getPositionError()) < distanceTolerance
                && Math.abs(angleController.getPositionError()) < rotationTolerance);
  }
}
