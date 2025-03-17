package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class QuestIORealV2 implements QuestIO {
  // Configure Network Tables topics (questnav/...) to communicate with the Quest
  private NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private NetworkTable nt4Table = nt4Instance.getTable("questnav");

  // Quest "output"?
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);

  // Quest "input"?
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables oculus data topics
  // Availabe frame data found here:
  // https://github.com/juchong/QuestNav/blob/main/unity/Assets/Robot/MotionStreamer.cs#L90
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0);
  private FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion =
      nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBattery = nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0);

  private Pose2d resetOculusPose;
  private Pose2d resetRobotPose;

  public QuestIORealV2() {
    zeroAbsolutePosition();
  }

  /** Zeroes the absolute 3D position of the robot (similar to long-pressing the quest logo) */
  private void zeroAbsolutePosition() {
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  public void updateInputs(QuestIOInputs inputs) {
    inputs.questPose = getQuestPose();
    inputs.robotPose = getRobotPose();
    inputs.rawPose = getUncorrectedOculusPose();

    // This changed in V0.7.0
    double timestamp = questTimestamp.getAtomic().serverTime;
    inputs.timestampDelta = timestamp - inputs.timestamp;
    inputs.timestamp = timestamp;

    inputs.batteryLevel = questBattery.get();

    // The timestamp delta is calculated between the current and last robot loop
    // The delta is zero if the new measurement is from the same time as the last measurement,
    // meaning we have not received new data and as such can assume the quest is not connected
    inputs.connected = inputs.timestampDelta != 0;
    cleanUpOculusMessages();
  }

  /** Sets supplied pose as origin of robot for calculations */
  public void resetPose(Pose2d robotPose) {
    resetOculusPose = getUncorrectedOculusPose();
    resetRobotPose = robotPose;
  }

  /**
   * IMPORTANT: Run periodically after processing.<br>
   * Cleans up Oculus subroutine messages after processing on the headset
   */
  private void cleanUpOculusMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  /**
   * Reads EulerAngles and calculates the assumed Yaw of the Oculus's pose. Input is in degrees.
   *
   * @return Rotation2d object of Oculus Yaw
   */
  private Rotation2d getUncorrectedOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    return Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));
  }

  /**
   * Reads the XY values of the Oculus on the field
   *
   * @return Translation2d representation of the Quest on the field
   */
  private Translation2d getUncorrectedOculusTranslation() {
    float[] questNavPosition = questPosition.get();
    return new Translation2d(questNavPosition[2], -questNavPosition[0]);
  }

  /**
   * Combines `getUncorrectedOculusTranslation` and `getUncorrectedOculusYaw` to get combined pose
   *
   * @return Uncorrected Oculus Pose on the field
   */
  private Pose2d getUncorrectedOculusPose() {
    return new Pose2d(getUncorrectedOculusTranslation(), getUncorrectedOculusYaw());
  }

  /**
   * To get the headset's actual pose on the field, you need to know: 1. Origin of the headset when
   * the match started 2. Origin of the robot when the match started 3. Transform of the "reset" and
   * "current" pose for the headset 4. The robot-to-field Pose when the headset was reset 5. The
   * robot-to-camera of the headset
   *
   * <p>Then you transform the "reset" robot pose by robot-to-camera and reset-to-current.
   *
   * @return Estimated pose of Oculus on the field
   */
  private Pose2d getQuestPose() {
    Transform2d poseRelativeToReset = getUncorrectedOculusPose().minus(resetOculusPose);
    return resetRobotPose
        .transformBy(QuestConstants.robotToQuestTransform)
        .transformBy(poseRelativeToReset);
  }

  /**
   * Reads the Quest's pose on the field from `getQuestPose` and transforms it to get the robot's pose.
   *
   * @return Estimated pose of the Robot on the field
   */
  public Pose2d getRobotPose() {
    return getQuestPose().transformBy(QuestConstants.robotToQuestTransform.inverse());
  }

  @Override
  public void close() {
    questMiso.close();
    questMosi.close();
    questTimestamp.close();
    questPosition.close();
    questQuaternion.close();
    questEulerAngles.close();
    questBattery.close();
  }
}
