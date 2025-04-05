package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseCommand extends Command {
  private final ProfiledPIDController perpendicularController =
      DriveCommandConstants.makeTranslationController();
  private final ProfiledPIDController parallelController =
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

  @Override
  public void execute() {
    Pose2d robotPose = useConstrainedPose ? drive.getConstrainedPose() : drive.getGlobalPose();
    Pose2d targetPose = targetPoseSupplier.get();
    Logger.recordOutput("Commands/" + getName() + "/TargetPose", targetPose);

    Transform2d error = robotPose.minus(targetPose);
    Logger.recordOutput("Commands/" + getName() + "/Error", error);

    double perpendicularSpeed = perpendicularController.calculate(error.getX(), 0);
    perpendicularSpeed = !perpendicularController.atSetpoint() ? perpendicularSpeed : 0;

    double parallelSpeed = parallelController.calculate(error.getY(), 0);
    parallelSpeed = !parallelController.atSetpoint() ? parallelSpeed : 0;

    double angularSpeed = angleController.calculate(error.getRotation().getRadians(), 0);
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

    ChassisSpeeds speeds = new ChassisSpeeds(perpendicularSpeed, parallelSpeed, angularSpeed);

    drive.runVelocity(speeds);

    Logger.recordOutput("Commands/" + getName() + "/IsAtSetpoint", atSetpoint().getAsBoolean())
    Logger.recordOutput("Commands/" + getName() + "/PerpendincularAtSetpoint", perpendicularController.atSetpoint());
    Logger.recordOutput("Commands/" + getName() + "/ParallelAtSetpoint", parallelController.atSetpoint());
    Logger.recordOutput("Commands/" + getName() + "/AngleAtSetpoint", angleController.atSetpoint());

  }

  @Override
  public void end(boolean interrupt) {
    perpendicularController.reset(0);
    parallelController.reset(0);
    angleController.reset(0);
  }

  public Trigger atSetpoint() {
    return new Trigger(
        () ->
            perpendicularController.atSetpoint()
                && parallelController.atSetpoint()
                && angleController.atSetpoint());
  }
}
