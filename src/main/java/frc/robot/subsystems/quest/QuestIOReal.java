package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class QuestIOReal implements QuestIO {
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

  private Pose2d resetPose;
  private float yaw_offset = 0.0f;

  public QuestIOReal() {
    resetPose(new Pose2d());
  }

  public void updateInputs(QuestIOInputs inputs) {
    inputs.questPose = getQuestPose();
    inputs.robotPose = getRobotPose();
    inputs.questTranslation = getQuestTranslation();

    inputs.rawPose = getRawPose();

    double timestamp = questTimestamp.get();
    inputs.timestampDelta = timestamp - inputs.timestamp;
    inputs.timestamp = timestamp;

    inputs.batteryLevel = questBattery.get();

    // The timestamp delta is calculated between the current and last robot loop
    // The delta is zero if the new measurement is from the same time as the last measurement,
    // meaning we have not received new data and as such can assume the quest is not connected
    inputs.connected = inputs.timestampDelta != 0;
    cleanUpOculusMessages();
  }

  /** Sets supplied pose as origin of all calculations */
  public void resetPose(Pose2d pose) {
    zeroAbsolutePosition();
    resetPose = pose;
  }

  /** Zeroes the absolute 3D position of the robot (similar to long-pressing the quest logo) */
  private void zeroAbsolutePosition() {
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  private void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
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

  /** Gets the yaw Euler angle of the headset */
  private double getQuestYawRad() {
    float[] eulerAngles = questEulerAngles.get();
    float ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return Rotation2d.fromDegrees(ret).getRadians();
    // return -Math.toRadians(eulerAngles[1]); // may need MathUtil.angleModulus(), not sure
  }

  private Translation2d getQuestTranslation() {
    float[] oculusPosition = questPosition.get();
    // return new Translation2d(-oculusPosition[0], oculusPosition[2]);
    return new Translation2d(oculusPosition[2], -oculusPosition[0]);
  }

  private Pose2d getRawPose() {
    return new Pose2d(getQuestTranslation(), Rotation2d.fromRadians(getQuestYawRad()));
  }

  private Pose2d getQuestPose() {
    return new Pose2d(
        getQuestTranslation().minus(QuestConstants.robotToQuest),
        Rotation2d.fromRadians(getQuestYawRad()));
  }

  private Pose2d getRobotPose() {
    return new Pose2d(
        getQuestPose().minus(resetPose).getTranslation(), Rotation2d.fromRadians(getQuestYawRad()));

    // return new Pose2d(
    //     getQuestTranslation()
    //         .rotateAround(getQuestTranslation(), QuestConstants.robotToQuest.getRotation())
    //         .plus(QuestConstants.robotToQuest.getTranslation())
    //         .plus(resetPose.getTranslation())
    //         .rotateAround(resetPose.getTranslation(), resetPose.getRotation()),
    //     Rotation2d.fromRadians(getQuestYawRad()).plus(resetPose.getRotation()));

    // return getQuestPose()
    //     .transformBy(new Transform2d(resetPose.getTranslation(), Rotation2d.kZero));
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
