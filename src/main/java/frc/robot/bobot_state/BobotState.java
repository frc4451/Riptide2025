package frc.robot.bobot_state;

import edu.wpi.first.math.geometry.Pose2d;
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

  @Override
  public void periodic() {
    Logger.recordOutput(logRoot + "HPS Zone", FieldUtils.getHPSZone());
  }

  @Override
  public void simulationPeriodic() {}
}
