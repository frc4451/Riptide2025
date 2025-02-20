package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.field.ReefFaces;
import frc.robot.field.ReefPole;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.quest.Quest;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.modes.ShooterModes;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import frc.robot.util.PoseUtils;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final Drive drive;
  private final SuperStructure superStructure;
  private final Quest quest;

  public Autos(Drive drive, SuperStructure superStructure, Quest quest) {
    this.drive = drive;
    this.superStructure = superStructure;
    this.quest = quest;
  }

  // Routines
  public AutoRoutine twoMeters() {
    return basicRoutine(drive.autoFactory.newRoutine("2 Meters"), ChoreoPaths.TWO_METERS);
  }

  public AutoRoutine threeMeters() {
    return basicRoutine(drive.autoFactory.newRoutine("3 Meters"), ChoreoPaths.THREE_METERS);
  }

  public AutoRoutine fiveMeters() {
    return basicRoutine(drive.autoFactory.newRoutine("5 Meters"), ChoreoPaths.FIVE_METERS);
  }

  public AutoRoutine curvy() {
    return basicRoutine(drive.autoFactory.newRoutine("Curvy"), ChoreoPaths.CURVY);
  }

  public AutoRoutine magikarp() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Magikarp");

    AutoTrajectory trajectory = routine.trajectory(ChoreoPaths.START_MID_TO_G.name);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                resetOdometry(trajectory),
                followTrajectory(trajectory),
                Commands.parallel(
                    positionToPole(() -> ReefFaces.GH.get().leftPole),
                    score(SuperStructureModes.L4))));

    return routine;
  }

  // Helpers
  private Command followTrajectory(AutoTrajectory trajectory) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                Logger.recordOutput(
                    "Odometry/Choreo/Trajectory", trajectory.getRawTrajectory().getPoses())),
        trajectory.cmd());
  }

  private Command resetQuest() {
    return Commands.runOnce(() -> quest.resetPose(drive.getPose()));
  }

  private Command resetOdometry(AutoTrajectory trajectory) {
    return trajectory.resetOdometry().andThen(resetQuest());
  }

  private Command positionToPole(Supplier<ReefPole> pole) {
    return new DriveToPoseCommand(
        drive,
        () ->
            PoseUtils.plusRotation(
                pole.get().getPerpendicularOffsetPose(AutoConstants.reefScoreOffsetMeters),
                Rotation2d.kPi));
  }

  private Command score(SuperStructureModes mode) {
    return Commands.sequence(
            superStructure.setModeCommand(mode),
            Commands.waitUntil(superStructure.isAtMode()),
            Commands.sequence(
                superStructure.setShooterModeCommand(ShooterModes.SHOOT),
                Commands.sequence(
                    Commands.waitUntil(superStructure.isCoralIntaked().negate()),
                    Commands.waitSeconds(0.1)),
                superStructure.setShooterModeCommand(ShooterModes.NONE)),
            superStructure.setModeCommand(SuperStructureModes.TUCKED),
            Commands.waitUntil(superStructure.isAtMode()))
        .onlyIf(superStructure.isCoralIntaked());
  }

  // Routine Logic
  private AutoRoutine basicRoutine(AutoRoutine routine, ChoreoPaths path) {
    AutoTrajectory trajectory = routine.trajectory(path.name);

    routine
        .active()
        .onTrue(Commands.sequence(resetOdometry(trajectory), followTrajectory(trajectory)));

    return routine;
  }
}
