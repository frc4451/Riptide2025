package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.quest.QuestSubsystem;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.modes.ShooterModes;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final Drive drive;
  private final SuperStructure superStructure;
  private final QuestSubsystem quest;

  public Autos(Drive drive, SuperStructure superStructure, QuestSubsystem quest) {
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

  public AutoRoutine fish() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Fish");

    AutoTrajectory trajectory = routine.trajectory(ChoreoPaths.START_MID_TO_J.name);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                resetOdometry(trajectory),
                followTrajectory(trajectory),
                Commands.parallel(
                    new DriveToPoseCommand(
                        drive, () -> FieldUtils.getClosestReef().leftPole.getPose()),
                    Commands.sequence(
                        superStructure.setModeCommand(SuperStructureModes.L4),
                        Commands.waitUntil(superStructure.isAtMode()),
                        superStructure.setShooterModeCommand(ShooterModes.SHOOT),
                        Commands.waitUntil(superStructure.isCoralIntaked().negate())
                            .andThen(Commands.waitSeconds(0.1)),
                        superStructure.setShooterModeCommand(ShooterModes.NONE)))));

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

  // Routine Logic
  private AutoRoutine basicRoutine(AutoRoutine routine, ChoreoPaths path) {
    AutoTrajectory trajectory = routine.trajectory(path.name);

    routine
        .active()
        .onTrue(Commands.sequence(resetOdometry(trajectory), followTrajectory(trajectory)));

    return routine;
  }
}
