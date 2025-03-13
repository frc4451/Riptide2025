package frc.robot.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.field.HumanPlayerStations;
import frc.robot.field.ReefFaces;
import frc.robot.field.ReefPole;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.quest.Quest;
import frc.robot.subsystems.superstructure.SuperStructure;
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
    AutoTrajectory trajectory = routine.trajectory(ChoreoPaths.START_MID_TO_GL4.name);

    routine
        .active()
        .onTrue(
            Commands.sequence(
                resetAndFollowTrajectory(trajectory),
                Commands.parallel(
                    positionToPole(
                        () -> ReefFaces.GH.get().leftPole, AutoConstants.l4ReefOffsetMeters),
                    superStructure.score(SuperStructureModes.L4Coral))));

    return routine;
  }

  public AutoRoutine binacle() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Binacle");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                resetAndFollowTrajectory(routine.trajectory(ChoreoPaths.START_MID_TO_CL4.name)),
                Commands.deadline(
                    superStructure.score(SuperStructureModes.L4Coral),
                    positionToPole(
                        () -> ReefFaces.GH.get().leftPole, AutoConstants.l4ReefOffsetMeters)),
                followTrajectory(routine.trajectory(ChoreoPaths.CL4_TO_HPS_RIGHT.name)),
                Commands.deadline(
                    superStructure.intake().andThen(Commands.waitSeconds(1.0)),
                    positionToHPS(() -> HumanPlayerStations.RIGHT.get())),
                followTrajectory(routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_CL4.name)),
                Commands.deadline(
                    superStructure.score(SuperStructureModes.L4Coral),
                    positionToPole(
                        () -> ReefFaces.CD.get().leftPole, AutoConstants.l4ReefOffsetMeters))));

    return routine;
  }

  public AutoRoutine barbaracle() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Barbaracle");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                resetAndFollowTrajectory(routine.trajectory(ChoreoPaths.START_BOTTOM_TO_EL4.name)),
                Commands.deadline(
                    superStructure.score(SuperStructureModes.L4Coral),
                    positionToPole(
                        () -> ReefFaces.EF.get().leftPole, AutoConstants.l4ReefOffsetMeters)),
                followTrajectory(routine.trajectory(ChoreoPaths.EL4_TO_HPS_RIGHT.name)),
                Commands.deadline(
                    superStructure.intake().andThen(Commands.waitSeconds(1.0)),
                    positionToHPS(() -> FieldConstants.blueHPSDriverRight)),
                followTrajectory(routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_CL4.name)),
                Commands.deadline(
                    superStructure.score(SuperStructureModes.L4Coral),
                    positionToPole(
                        () -> ReefFaces.CD.get().leftPole, AutoConstants.l4ReefOffsetMeters))));

    return routine;
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
                alignAndScore(() -> ReefFaces.EF.get().rightPole, AutoConstants.l2ReefOffsetMeters),
                awayFromReef(
                    routine.trajectory(ChoreoPaths.FL2_TO_HPS_RIGHT.name),
                    SuperStructureModes.TUCKED),
                superStructure.intake(),
                pathAndMode(
                    routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_EL2.name),
                    SuperStructureModes.L2Coral),
                alignAndScore(
                    () -> ReefFaces.EF.get().leftPole, AutoConstants.l2ReefOffsetMeters)));

    return routine;
  }

  public AutoRoutine allredL4() {
    AutoRoutine routine = drive.autoFactory.newRoutine("Allred L4");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.START_BOTTOM_TO_FL4.name)),
                superStructure.setModeAndWaitCommand(SuperStructureModes.L4Coral),
                alignAndScoreNew(
                    () -> ReefFaces.EF.get().rightPole, AutoConstants.l4ReefOffsetMeters),
                backupFromReef(() -> ReefFaces.EF.get().rightPole),
                // HPS
                delayedTuckAndGo(routine.trajectory(ChoreoPaths.FL4_TO_HPS_RIGHT.name)),
                superStructure.intake(),
                // L4
                prepAndGo(routine.trajectory(ChoreoPaths.HPS_RIGHT_TO_CL4.name)),
                superStructure.setModeAndWaitCommand(SuperStructureModes.L4Coral),
                alignAndScoreNew(
                    () -> ReefFaces.CD.get().leftPole, AutoConstants.l4ReefOffsetMeters),
                backupFromReef(() -> ReefFaces.CD.get().leftPole)));

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
                    positionToPole(
                        () -> ReefFaces.EF.get().leftPole, AutoConstants.l4ReefOffsetMeters))
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

  private Command resetQuest() {
    return Commands.runOnce(() -> quest.resetPose(drive.getPose()));
  }

  private Command resetOdometry(AutoTrajectory trajectory) {
    return trajectory.resetOdometry().andThen(resetQuest());
  }

  private Command resetAndFollowTrajectory(AutoTrajectory trajectory) {
    return Commands.sequence(resetOdometry(trajectory), followTrajectory(trajectory));
  }

  private Command positionToPole(Supplier<ReefPole> pole, double reefOffsetMeters) {
    return new DriveToPoseCommand(
        drive,
        () ->
            PoseUtils.plusRotation(
                pole.get().getPerpendicularOffsetPose(reefOffsetMeters), Rotation2d.kPi));
  }

  private Command positionToHPS(Supplier<AprilTagStruct> hps) {
    return new DriveToPoseCommand(
        drive,
        () ->
            PoseUtils.getPerpendicularOffsetPose(
                hps.get().pose().toPose2d(), AutoConstants.l2ReefOffsetMeters));
  }

  private Command pathAndMode(AutoTrajectory trajectory, SuperStructureModes mode) {
    return Commands.deadline(followTrajectory(trajectory), superStructure.setModeCommand(mode));
  }

  // Classic sequencing
  private Command alignAndScore(Supplier<ReefPole> poleSupplier, double reefOffsetMeters) {
    return Commands.deadline(
        Commands.waitSeconds(1),
        Commands.sequence(Commands.waitSeconds(0.5), superStructure.shootCoral()),
        positionToPole(poleSupplier, reefOffsetMeters));
  }

  private Command awayFromReef(AutoTrajectory trajectory, SuperStructureModes mode) {
    return Commands.deadline(
        followTrajectory(trajectory),
        Commands.sequence(Commands.waitSeconds(0.3), superStructure.setModeCommand(mode)));
  }

  // New Sequencing
  private Command positionToPoleAndWait(Supplier<ReefPole> pole, double reefOffsetMeters) {
    DriveToPoseCommand cmd =
        new DriveToPoseCommand(
            drive,
            () ->
                PoseUtils.plusRotation(
                    pole.get().getPerpendicularOffsetPose(reefOffsetMeters), Rotation2d.kPi));

    return cmd.until(cmd.atSetpoint()).unless(cmd.atSetpoint());
  }

  private Command alignAndScoreNew(Supplier<ReefPole> poleSupplier, double reefOffsetMeters) {
    return Commands.sequence(
        positionToPoleAndWait(poleSupplier, reefOffsetMeters), superStructure.shootCoral());
  }

  private Command backupFromReef(Supplier<ReefPole> pole) {
    return Commands.sequence(
        positionToPoleAndWait(() -> pole.get(), AutoConstants.elevatorDownOffsetMeters),
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
        Commands.waitSeconds(1.575)
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
