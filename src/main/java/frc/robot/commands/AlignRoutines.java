package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoConstants;
import frc.robot.field.ReefPole;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AlignRoutines {
  public static DriveToPoseCommand positionToPole(
      Drive drive, Supplier<ReefPole> pole, DoubleSupplier reefOffsetMeters) {
    return new DriveToPoseCommand(
        drive,
        AutoConstants.useConstrainedPoseForReef,
        () ->
            pole.get()
                .getPerpendicularOffsetPose(reefOffsetMeters.getAsDouble())
                .transformBy(new Transform2d(0, 0, Rotation2d.kPi)));
  }

  public static Command positionToPoleUntilDone(
      Drive drive, Supplier<ReefPole> pole, DoubleSupplier reefOffsetMeters) {
    DriveToPoseCommand cmd = positionToPole(drive, pole, reefOffsetMeters);

    return cmd.until(cmd.atSetpoint()).unless(cmd.atSetpoint());
  }

  /**
    * {@link SuperStructureModes} for {@link SuperStructure} must be set beforehand
    */
  public static Command positionToPoleAndScore(
      Drive drive,
      SuperStructure superStructure,
      Supplier<ReefPole> poleSupplier,
      DoubleSupplier reefOffsetMeters) {
    return Commands.sequence(
        positionToPoleUntilDone(drive, poleSupplier, reefOffsetMeters),
        Commands.deadline(
            Commands.sequence(
                Commands.waitUntil(superStructure.isAtMode()), superStructure.shootCoral()),
            positionToPole(drive, poleSupplier, reefOffsetMeters)));
  }

  public static DrivePerpendicularToPoseCommand alignToPose(
      Drive drive, Supplier<Pose2d> targetSupplier, DoubleSupplier perpendicularInput) {
    return new DrivePerpendicularToPoseCommand(
        drive, AutoConstants.useConstrainedPoseForReef, targetSupplier, perpendicularInput);
  }
}
