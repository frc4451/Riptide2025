package frc.robot.bobot_state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state.varc.BargeTagTracker;
import frc.robot.bobot_state.varc.HPSTagTracker;
import frc.robot.bobot_state.varc.ReefTagTracker;
import frc.robot.bobot_state.varc.TargetAngleTracker;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.quest.TimestampedPose;
import frc.robot.subsystems.vision.PoseObservation;
import frc.robot.util.PoseUtils;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import org.littletonrobotics.junction.Logger;

/**
 * Class full of static variables and methods that store robot state we'd need across mulitple
 * subsystems. It's called {@link #BobotState} as to not conflict with WPILib's {@link
 * edu.wpi.first.wpilibj.RobotState}
 */
public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  private static final Queue<PoseObservation> globalPoseObservations =
      new LinkedBlockingQueue<>(20);
  private static final Queue<PoseObservation> constrainedPoseObservations =
      new LinkedBlockingQueue<>(20);
  private static final Queue<TimestampedPose> questMeasurements = new LinkedBlockingQueue<>(20);

  private static Pose2d globalPose = new Pose2d();
  private static Pose2d constrainedPose = new Pose2d();
  private static Pose2d questPose = new Pose2d();

  public static final ReefTagTracker reefTracker = new ReefTagTracker();
  public static final HPSTagTracker hpsTracker = new HPSTagTracker();
  public static final BargeTagTracker bargeTracker = new BargeTagTracker();

  public static boolean climbMode = false;

  private static List<TargetAngleTracker> autoAlignmentTrackers =
      List.of(BobotState.hpsTracker, BobotState.reefTracker);

  public static void offerGlobalVisionObservation(PoseObservation observation) {
    BobotState.globalPoseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getGlobalVisionObservations() {
    return BobotState.globalPoseObservations;
  }

  public static void offerConstrainedVisionObservation(PoseObservation observation) {
    BobotState.constrainedPoseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getConstrainedVisionObservations() {
    return BobotState.constrainedPoseObservations;
  }

  public static void offerQuestMeasurement(TimestampedPose observation) {
    BobotState.questMeasurements.offer(observation);
  }

  public static Queue<TimestampedPose> getQuestMeasurments() {
    return BobotState.questMeasurements;
  }

  public static void updateGlobalPose(Pose2d pose) {
    BobotState.globalPose = pose;
  }

  public static void updateConstrainedPose(Pose2d pose) {
    BobotState.constrainedPose = pose;
  }

  public static void updateQuestPose(Pose2d pose) {
    BobotState.questPose = pose;
  }

  public static Pose2d getGlobalPose() {
    return BobotState.globalPose;
  }

  public static Pose2d getConstrainedPose() {
    return BobotState.constrainedPose;
  }

  public static Pose2d getQuestPose() {
    return BobotState.questPose;
  }

  public static Trigger autoAlignEnabled() {
    return new Trigger(() -> FieldUtils.onAllianceSide(globalPose, FieldConstants.bargeLength));
  }

  public static Trigger humanPlayerShouldThrow() {
    return new Trigger(
        () ->
            PoseUtils.getPerpendicularError(
                    BobotState.getGlobalPose(), FieldUtils.getClosestHPS().center)
                < 0.5);
  }

  public static TargetAngleTracker getCurrentAlignmentTracker() {
    return climbMode
        ? bargeTracker
        : autoAlignmentTrackers.stream()
            .reduce((a, b) -> a.getDistanceMeters() < b.getDistanceMeters() ? a : b)
            .get();
  }

  @Override
  public void periodic() {
    Logger.recordOutput(logRoot + "ClimberMode", climbMode);

    {
      TimestampedPose[] questPoses = getQuestMeasurments().stream().toArray(TimestampedPose[]::new);
      Logger.recordOutput(logRoot + "Quest/Measurements", questPoses);
    }

    {
      reefTracker.update();

      String calcLogRoot = logRoot + "Reef/";
      Logger.recordOutput(calcLogRoot + "ClosestTag", FieldUtils.getClosestReef().tag);
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", reefTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", reefTracker.getRotationTarget().getRadians());
      Logger.recordOutput(calcLogRoot + "Left Pole", FieldUtils.getClosestReef().leftPole);
      Logger.recordOutput(calcLogRoot + "Right Pole", FieldUtils.getClosestReef().rightPole);
    }

    {
      hpsTracker.update();

      String calcLogRoot = logRoot + "HPS/";
      Logger.recordOutput(calcLogRoot + "Closest Tag", FieldUtils.getClosestHPS().tag);
      Logger.recordOutput(calcLogRoot + "Distance", BobotState.hpsTracker.getDistanceMeters());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", hpsTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", hpsTracker.getRotationTarget().getRadians());
    }

    {
      bargeTracker.update();

      String calcLogRoot = logRoot + "Barge/";
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", hpsTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", hpsTracker.getRotationTarget().getRadians());
    }

    {
      String calcLogRoot = logRoot + "CurrentAlignment/";
      Logger.recordOutput(calcLogRoot + "Enabled", autoAlignEnabled().getAsBoolean());
      Logger.recordOutput(
          calcLogRoot + "Type", getCurrentAlignmentTracker().getClass().getSimpleName());
    }

    Logger.recordOutput(logRoot + "OnAllianceSide", FieldUtils.onAllianceSide(globalPose, 0));
  }

  @Override
  public void simulationPeriodic() {}
}
