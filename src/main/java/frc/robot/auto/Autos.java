package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.AlignRoutines;
import frc.robot.field.FieldConstants;
import frc.robot.field.HumanPlayerStations;
import frc.robot.field.ReefFaces;
import frc.robot.field.ReefPole;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Autos {
  private final Drive drive;
  private final SuperStructure superStructure;

  public Autos(Drive drive, SuperStructure superStructure) {
    this.drive = drive;
    this.superStructure = superStructure;
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

  public AutoRoutine allredL2() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Allred L2");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                pathAndMode(
                    routine.trajectory(ChoreoPaths.START_BOTTOM_TO_FL2.name),
                    SuperStructureModes.L2Coral),
                alignAndScore(
                    () -> ReefFaces.EF.get().rightPole, FieldConstants.eventConstants.l2ReefOffset),
                awayFromReef(
                    routine.trajectory(ChoreoPaths.FL2_TO_HPS_RIGHT.name),
                    SuperStructureModes.TUCKED),
                superStructure.intake(),
                pathAndMode(
                    routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_EL2.name),
                    SuperStructureModes.L2Coral),
                alignAndScore(
                    () -> ReefFaces.EF.get().leftPole,
                    FieldConstants.eventConstants.l2ReefOffset)));

    return routine;
  }

  public AutoRoutine allLeftL4() {
    AutoRoutine routine = drive.autoFactory.newRoutine("AllLeft L4");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.START_TOP_TO_IL4.name)),
                superStructure.setModeCommand(SuperStructureModes.L4Coral),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.IJ.get().leftPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                backupFromReef(() -> ReefFaces.IJ.get().leftPole),
                // HPS
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.IL4_TO_HPS_LEFT.name)),
                Commands.deadline(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(
                        drive, () -> HumanPlayerStations.RIGHT.get())),
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.HPS_LEFT_TO_LL4.name)),
                superStructure.setModeCommand(SuperStructureModes.L4Coral),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.KL.get().rightPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                backupFromReef(() -> ReefFaces.KL.get().rightPole),
                // HPS
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.LL4_TO_HPS_LEFT.name)),
                Commands.parallel(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(
                        drive, () -> HumanPlayerStations.RIGHT.get()))));

    return routine;
  }

  public AutoRoutine allRightL4() {
    AutoRoutine routine = drive.autoFactory.newRoutine("AllRight L4");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.START_BOTTOM_TO_FL4.name)),
                superStructure.setModeCommand(SuperStructureModes.L4Coral),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.EF.get().rightPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                backupFromReef(() -> ReefFaces.EF.get().rightPole),
                // HPS
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.FL4_TO_HPS_RIGHT.name)),
                Commands.deadline(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(
                        drive, () -> HumanPlayerStations.RIGHT.get())),
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_CL4.name)),
                superStructure.setModeCommand(SuperStructureModes.L4Coral),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.CD.get().leftPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                backupFromReef(() -> ReefFaces.CD.get().leftPole),
                // HPS
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.CL4_TO_HPS_RIGHT.name)),
                Commands.parallel(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(
                        drive, () -> HumanPlayerStations.RIGHT.get()))));

    return routine;
  }

  public AutoRoutine tripleThreat() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Triple Threat");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                resetAndFollowTrajectory(routine.trajectory(ChoreoPaths.START_BOTTOM_TO_EL4.name)),
                Commands.deadline(
                    superStructure.score(SuperStructureModes.L4Coral),
                    AlignRoutines.positionToPole(
                        drive,
                        () -> ReefFaces.EF.get().leftPole,
                        () -> FieldConstants.eventConstants.l4ReefOffset))
                // followTrajectory(routine.trajectory(ChoreoPaths.E_TO_HPS_RIGHT.name)),
                // followTrajectory(routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_D.name)),
                // followTrajectory(routine.trajectory(ChoreoPaths.D_TO_HPS_RIGHT.name)),
                // followTrajectory(routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_C.name))
                ));

    return routine;
  }

  // Helpers
  private Command followTrajectory(AutoTrajectory trajectory) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                Logger.recordOutput(
                    "Odometry/Choreo/Trajectory", trajectory.getRawTrajectory().getPoses())),
        trajectory.cmd(),
        Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive));
  }

  private Command resetOdometry(AutoTrajectory trajectory) {
    return trajectory.resetOdometry();
  }

  private Command resetAndFollowTrajectory(AutoTrajectory trajectory) {
    return Commands.sequence(resetOdometry(trajectory), followTrajectory(trajectory));
  }

  private Command pathAndMode(AutoTrajectory trajectory, SuperStructureModes mode) {
    return Commands.deadline(followTrajectory(trajectory), superStructure.setModeCommand(mode));
  }

  // Classic sequencing
  /**
   * @deprecated
   * Use {@link AlignRoutines#positionToPoleAndScore(Drive, SuperStructure, SuperStructureModes, Supplier, java.util.function.DoubleSupplier) instead
   */
  @Deprecated
  private Command alignAndScore(Supplier<ReefPole> poleSupplier, double reefOffsetMeters) {
    return Commands.deadline(
        Commands.waitSeconds(1),
        Commands.sequence(Commands.waitSeconds(0.5), superStructure.shootCoral()),
        AlignRoutines.positionToPole(drive, poleSupplier, () -> reefOffsetMeters));
  }

  /**
   * @deprecated
   * Use {@link #backupFromReef(Supplier) instead
   */
  @Deprecated
  private Command awayFromReef(AutoTrajectory trajectory, SuperStructureModes mode) {
    return Commands.deadline(
        followTrajectory(trajectory),
        Commands.sequence(Commands.waitSeconds(0.3), superStructure.setModeCommand(mode)));
  }

  // New Sequencing
  private Command backupFromReef(Supplier<ReefPole> pole) {
    return Commands.sequence(
        AlignRoutines.positionToPoleUntilDone(
            drive, () -> pole.get(), () -> FieldConstants.eventConstants.elevatorDownOffset),
        superStructure.setModeAndWaitCommand(SuperStructureModes.TUCKED_L4));
  }

  private Command prepAndGo(AutoTrajectory trajectory) {
    return pathAndMode(trajectory, SuperStructureModes.TUCKED_L4);
  }

  private Command tuckAndGo(AutoTrajectory trajectory) {
    return pathAndMode(trajectory, SuperStructureModes.TUCKED);
  }

  private Command delayedTuckAndGo(AutoTrajectory trajectory) {
    return Commands.deadline(
        followTrajectory(trajectory),
        Commands.waitUntil(
                () ->
                    BobotState.reefTracker.getDistanceMeters()
                        > AutoConstants.tuckReefOffsetThresholdMeters)
            .andThen(superStructure.setModeCommand(SuperStructureModes.TUCKED)));
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
