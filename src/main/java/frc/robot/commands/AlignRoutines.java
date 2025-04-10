package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoConstants;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.field.HumanPlayerStation;
import frc.robot.field.ReefPole;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.subsystems.superstructure.shooter.ShooterModes;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AlignRoutines {
  public static final double distanceShootTolerance = Units.inchesToMeters(0.5);
  public static final double rotationShootTolerance = Units.degreesToRadians(1);
  public static final double velocityTolerance = 0.2;

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

  public static Command positionToPoleUntilCloseEnough(
      Drive drive, Supplier<ReefPole> pole, DoubleSupplier reefOffsetMeters) {
    DriveToPoseCommand cmd = positionToPole(drive, pole, reefOffsetMeters);

    Trigger canShoot =
        cmd.canShoot(distanceShootTolerance, rotationShootTolerance, velocityTolerance);

    return cmd.until(canShoot).unless(canShoot);
  }

  /** {@link SuperStructureModes} for {@link SuperStructure} must be set beforehand */
  public static Command positionToPoleAndScore(
      Drive drive,
      SuperStructure superStructure,
      Supplier<ReefPole> poleSupplier,
      DoubleSupplier reefOffsetMeters) {
    return Commands.sequence(
        positionToPoleUntilCloseEnough(drive, poleSupplier, reefOffsetMeters),
        Commands.deadline(
            Commands.sequence(
                Commands.waitUntil(() -> superStructure.shouldShootCoral()),
                superStructure.shootCoral()),
            positionToPole(drive, poleSupplier, reefOffsetMeters)));
  }

  public static Command positionToPoleAndAlgae(
      Drive drive, SuperStructure superStructure, Supplier<ReefPole> poleSupplier) {
    return Commands.sequence(
        superStructure.setModeAndWaitCommand(SuperStructureModes.L2Algae),
        superStructure.setShooterModeCommand(ShooterModes.ALGAE_INTAKING),
        positionToPoleUntilDone(
            drive, poleSupplier, () -> FieldConstants.eventConstants.algaeOffset));
  }

  public static DriveToPoseCommand positionToPose(Drive drive, Supplier<Pose2d> targetSupplier) {
    return new DriveToPoseCommand(drive, false, targetSupplier);
  }

  public static DrivePerpendicularToPoseCommand alignToPose(
      Drive drive, Supplier<Pose2d> targetSupplier, DoubleSupplier perpendicularInput) {
    return new DrivePerpendicularToPoseCommand(drive, false, targetSupplier, perpendicularInput);
  }

  public static Command positionToTag(
      Drive drive,
      Supplier<AprilTagStruct> tag,
      DoubleSupplier perpendicularDistance,
      DoubleSupplier parallelDistance) {
    return new DriveToPoseCommand(
        drive,
        false,
        () ->
            tag.get()
                .pose()
                .toPose2d()
                .transformBy(
                    new Transform2d(
                        perpendicularDistance.getAsDouble(),
                        parallelDistance.getAsDouble(),
                        Rotation2d.kZero)));
  }

  public static Command positionToHPSCenter(Drive drive, Supplier<HumanPlayerStation> hps) {
    return new DriveToPoseCommand(drive, false, () -> hps.get().center);
  }

  public static Command positionToHPSClosest(Drive drive, Supplier<HumanPlayerStation> hps) {
    return new DriveToPoseCommand(drive, false, () -> hps.get().getClosest(drive.getGlobalPose()));
  }
}
