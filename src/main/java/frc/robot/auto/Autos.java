package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.AlignRoutines;
import frc.robot.field.Barge;
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

  //private final AutoRoutine callahan;

  public Autos(Drive drive, SuperStructure superStructure) {
    this.drive = drive;
    this.superStructure = superStructure;

    drive.autoFactory.bind("L4", superStructure.setModeCommand(SuperStructureModes.L4Coral));

    // this.callahan = drive.autoFactory.newRoutine("Callahan");
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
                logRoutine("Allred L2"),
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
                logRoutine("AllLeft L4"),
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
                    AlignRoutines.positionToHPSCenter(drive, () -> HumanPlayerStations.LEFT.get())),
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
                        drive, () -> HumanPlayerStations.LEFT.get()))));

    return routine;
  }

  public AutoRoutine allRightL4() {
    AutoRoutine routine = drive.autoFactory.newRoutine("AllRight L4");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                logRoutine("AllRight L4"),
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

  public AutoRoutine callahan() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Callahan");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                logRoutine("Callahan"),
                // L4
                superStructure.setModeAndWaitCommand(SuperStructureModes.TUCKED_L4),
                prepAndGo(routine.trajectory(ChoreoPaths.START_RIGHT_TO_FL4_NO_STOP.name)),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.EF.get().rightPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                // HPS
                superStructure.setModeCommand(SuperStructureModes.TUCKED_L4),
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.FL4_TO_HPS_RIGHT_NO_STOP.name)),
                Commands.deadline(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(
                        drive, () -> HumanPlayerStations.RIGHT.get())),
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_CL4_NO_STOP.name)),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.CD.get().leftPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                // HPS
                superStructure.setModeCommand(SuperStructureModes.TUCKED_L4),
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.CL4_TO_HPS_RIGHT_NO_STOP.name)),
                Commands.deadline(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(
                        drive, () -> HumanPlayerStations.RIGHT.get())),
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_DL4_NO_STOP.name)),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.CD.get().rightPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                // HPS
                superStructure.setModeCommand(SuperStructureModes.TUCKED_L4),
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.DL4_TO_HPS_RIGHT_NO_STOP.name)),
                Commands.deadline(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(drive, () -> HumanPlayerStations.RIGHT.get()))
                //
                ));

    return routine;
  }

  public AutoRoutine ethan() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Ethan");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                logRoutine("Ethan"),
                // L4
                superStructure.setModeAndWaitCommand(SuperStructureModes.TUCKED_L4),
                prepAndGo(routine.trajectory(ChoreoPaths.START_LEFT_TO_IL4_NO_STOP.name)),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.IJ.get().leftPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                // HPS
                superStructure.setModeCommand(SuperStructureModes.TUCKED_L4),
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.IL4_TO_HPS_LEFT_NO_STOP.name)),
                Commands.deadline(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(drive, () -> HumanPlayerStations.LEFT.get())),
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.HPS_LEFT_TO_LL4_NO_STOP.name)),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.KL.get().rightPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                // HPS
                superStructure.setModeCommand(SuperStructureModes.TUCKED_L4),
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.LL4_TO_HPS_LEFT_NO_STOP.name)),
                Commands.deadline(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(drive, () -> HumanPlayerStations.LEFT.get())),
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.HPS_LEFT_TO_KL4_NO_STOP.name)),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.KL.get().leftPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                // HPS
                superStructure.setModeCommand(SuperStructureModes.TUCKED_L4),
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.KL4_TO_HPS_LEFT_NO_STOP.name)),
                Commands.deadline(
                    superStructure.intake(),
                    AlignRoutines.positionToHPSCenter(drive, () -> HumanPlayerStations.LEFT.get()))
                //
                ));
    return routine;
  }

  public AutoRoutine algae() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Algae");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                logRoutine("Algae"),
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.START_MID_TO_GL4.name)),
                superStructure.setModeCommand(SuperStructureModes.L4Coral),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.GH.get().leftPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                // Grab Algae
                backupForAlgae(() -> ReefFaces.GH.get().center, SuperStructureModes.L2Algae),
                AlignRoutines.positionToPoleAndAlgae(
                    drive,
                    superStructure,
                    SuperStructureModes.L2Algae,
                    () -> ReefFaces.GH.get().center),
                backupForBarge(() -> ReefFaces.GH.get().center),
                // Score Algae
                AlignRoutines.positionToBargeAndScore(drive, superStructure),
                backoffBargeForAutoRP()
                //
                ));

    return routine;
  }

  public AutoRoutine algaeTwo() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Algae 2");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                logRoutine("Algae 2"),
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.START_MID_TO_GL4.name)),
                superStructure.setModeCommand(SuperStructureModes.L4Coral),
                AlignRoutines.positionToPoleAndScore(
                    drive,
                    superStructure,
                    () -> ReefFaces.GH.get().leftPole,
                    () -> FieldConstants.eventConstants.l4ReefOffset),
                // Grab Algae
                backupForAlgae(() -> ReefFaces.GH.get().center, SuperStructureModes.L2Algae),
                AlignRoutines.positionToPoleAndAlgae(
                    drive,
                    superStructure,
                    SuperStructureModes.L2Algae,
                    () -> ReefFaces.GH.get().center),
                backupForBarge(() -> ReefFaces.GH.get().center),
                // Score Algae
                AlignRoutines.positionToBargeAndScore(drive, superStructure),
                superStructure.setModeAndWaitCommand(SuperStructureModes.TUCKED_L4),
                // Grab Algae
                backupForAlgae(() -> ReefFaces.IJ.get().center, SuperStructureModes.L3Algae),
                AlignRoutines.positionToPoleAndAlgae(
                    drive,
                    superStructure,
                    SuperStructureModes.L3Algae,
                    () -> ReefFaces.IJ.get().center),
                backupForBarge(() -> ReefFaces.IJ.get().center),
                // Score Algae
                AlignRoutines.positionToBargeAndScore(drive, superStructure),
                backoffBargeForAutoRP()
                //
                ));

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
    return Commands.parallel(
        Commands.runOnce(
            () ->
                Logger.recordOutput(
                    "Odometry/Choreo/Trajectory", trajectory.getRawTrajectory().getPoses())),
        Commands.sequence(
            trajectory.cmd(),
            Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive)));
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
    return Commands.race(
        AlignRoutines.positionToPoleUntilCloseEnough(
            drive, () -> pole.get(), () -> FieldConstants.eventConstants.elevatorDownOffset),
        superStructure.setModeAndWaitCommand(SuperStructureModes.TUCKED_L4));
  }

  private Command backupForAlgae(Supplier<ReefPole> pole, SuperStructureModes mode) {
    return Commands.sequence(
        AlignRoutines.positionToPoleUntilDone(drive, () -> pole.get(), () -> Units.feetToMeters(3)),
        superStructure.setModeAndWaitCommand(mode));
  }

  private Command backupForBarge(Supplier<ReefPole> pole) {
    return Commands.sequence(
        AlignRoutines.positionToPoleUntilDone(
            drive, () -> pole.get(), () -> Units.feetToMeters(3.1)),
        superStructure.setModeCommand(SuperStructureModes.TUCKED_L4));
  }

  private Command backoffBargeForAutoRP() {
    return Commands.sequence(
        Commands.parallel(
            AlignRoutines.positionToPose(
                drive,
                () ->
                    Barge.get()
                        .shot
                        .transformBy(
                            new Transform2d(Units.inchesToMeters(12), 0.8, Rotation2d.kPi))),
            superStructure.setModeAndWaitCommand(SuperStructureModes.TUCKED_L4)),
        superStructure.setModeCommand(SuperStructureModes.TUCKED));
  }

  private Command logRoutine(String name) {
    // AutoRoutine.name should be public chat
    return Commands.runOnce(() -> Logger.recordOutput("Choreo/Routine", name));
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
        Commands.sequence(
            Commands.waitUntil(
                () ->
                    BobotState.reefTracker.getDistanceMeters()
                        > AutoConstants.tuckReefOffsetThresholdMeters),
            superStructure.setModeCommand(SuperStructureModes.TUCKED),
            superStructure.intake()));
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
