package frc.robot.bobot_state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.bobot_state.varc.HPSTagTracker;
import frc.robot.bobot_state.varc.ReefTagTracker;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.quest.TimestampedPose;
import frc.robot.subsystems.vision.PoseObservation;
import frc.robot.util.VirtualSubsystem;
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

  private static final Queue<PoseObservation> poseObservations = new LinkedBlockingQueue<>(20);
  private static final Queue<TimestampedPose> questMeasurements = new LinkedBlockingQueue<>(20);

  private static Pose2d globalPose = new Pose2d();

  private static ReefTagTracker reefTracker = new ReefTagTracker();
  private static HPSTagTracker hpsTracker = new HPSTagTracker();

  public static void offerVisionObservation(PoseObservation observation) {
    BobotState.poseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getVisionObservations() {
    return BobotState.poseObservations;
  }

  public static void offerQuestMeasurment(TimestampedPose observation) {
    BobotState.questMeasurements.offer(observation);
  }

  public static Queue<TimestampedPose> getQuestMeasurments() {
    return BobotState.questMeasurements;
  }

  public static void updateGlobalPose(Pose2d pose) {
    BobotState.globalPose = pose;
  }

  public static Pose2d getGlobalPose() {
    return BobotState.globalPose;
  }

  public static Rotation2d getRotationToClosestReefIfPresent() {
    return BobotState.reefTracker.getRotationTarget().orElse(BobotState.globalPose.getRotation());
  }

  public static Rotation2d getRotationToClosestHPSfIfPresent() {
    return BobotState.hpsTracker.getRotationTarget().orElse(BobotState.globalPose.getRotation());
  }

  @Override
  public void periodic() {

    {
      reefTracker.update();

      String calcLogRoot = logRoot + "Reef/";
      Logger.recordOutput(calcLogRoot + "Closest Tag", FieldUtils.getClosestReefAprilTag());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg",
          reefTracker.getRotationTarget().map(Rotation2d::getDegrees).orElse(Double.NaN));
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad",
          reefTracker.getRotationTarget().map(Rotation2d::getRadians).orElse(Double.NaN));
    }

    {
      hpsTracker.update();

      String calcLogRoot = logRoot + "HPS/";
      Logger.recordOutput(calcLogRoot + "Closest Tag", FieldUtils.getClosestHPSTag());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg",
          hpsTracker.getRotationTarget().map(Rotation2d::getDegrees).orElse(Double.NaN));
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad",
          hpsTracker.getRotationTarget().map(Rotation2d::getRadians).orElse(Double.NaN));
    }
  }

  @Override
  public void simulationPeriodic() {}
}
