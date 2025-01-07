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

class QuestReal implements QuestIO {
    // Configure Network Tables topics (oculus/...) to communicate with the Quest
    private NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    private NetworkTable nt4Table = nt4Instance.getTable("oculus");

    // Quest "output"?
    private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);

    // Quest "input"?
    private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

    // Subscribe to the Network Tables oculus data topics
    // Availabe frame data found here:
    // https://github.com/juchong/QuestNav/blob/main/unity/Assets/Robot/MotionStreamer.cs#L90
    private DoubleSubscriber questTimestamp = nt4Table
            .getDoubleTopic("timestamp")
            .subscribe(0.0f);
    private FloatArraySubscriber questPosition = nt4Table
            .getFloatArrayTopic("position")
            .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
    private FloatArraySubscriber questQuaternion = nt4Table
            .getFloatArrayTopic("quaternion")
            .subscribe(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
    private FloatArraySubscriber questEulerAngles = nt4Table
            .getFloatArrayTopic("eulerAngles")
            .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
    private DoubleSubscriber questBattery = nt4Table
            .getDoubleTopic("batteryLevel")
            .subscribe(0.0f);

    /*
     * These are used to offset the headset's origin (0, 0) so that wherever the
     * robot is recentered will match with the expected field pose.
     */
    private Translation2d translationOffset = new Translation2d();
    private double yawOffsetRad = 0.0;

    public void updateInputs(QuestIOInputs inputs) {
        inputs.pose = getPose();
        inputs.yawRad = getYawRad();

        inputs.timestamp = questTimestamp.get();
        inputs.batteryLevel = questBattery.get();

        inputs.rawPose = getRawPose();
        inputs.rawYawRad = getRawYawRad();

        inputs.rawPosition = questPosition.get();
        inputs.rawQuaternion = questQuaternion.get();

        cleanUpOculusMessages();
    }

    /** Sets supplied pose as origin of all calculations */
    public void resetPose(Pose2d pose) {
        translationOffset = pose.getTranslation();
        yawOffsetRad = pose.getRotation().getRadians();
        zeroAbsolutePosition();
    }

    /**
     * Zeroes the absolute 3D position of the robot
     * (similar to long-pressing the quest logo)
     */
    private void zeroAbsolutePosition() {
        if (questMiso.get() != 99) {
            questMosi.set(1);
        }
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
     * Gets the yaw Euler angle of the headset
     */
    private double getRawYawRad() {
        float[] eulerAngles = questEulerAngles.get();
        return Math.toRadians(eulerAngles[1]); // may need MathUtil.angleModulus(), not sure
    }

    /**
     * Gets the yaw Euler angle of the headset with yaw offset applied
     */
    private double getYawRad() {
        return getRawYawRad() - yawOffsetRad; // may need MathUtil.angleModulus(), not sure
    }

    private Translation2d getRawTranslation() {
        float[] oculusPosition = questPosition.get();
        return new Translation2d(oculusPosition[2], -oculusPosition[0]);
    }

    private Translation2d getTranslation() {
        return getRawTranslation().plus(translationOffset);
    }

    private Pose2d getRawPose() {
        return new Pose2d(getRawTranslation(), Rotation2d.fromRadians(getRawYawRad()));
    }

    private Pose2d getPose() {
        return new Pose2d(getTranslation(), Rotation2d.fromRadians(getYawRad()));
    }
}
