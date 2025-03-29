package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Quest extends VirtualSubsystem {
  private final QuestIO io;
  private final QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Quest Disconnected!", AlertType.kWarning);
  private final Alert lowBatteryAlert =
      new Alert("Quest Battery is Low! (<25%)", AlertType.kWarning);

  private final QuestCalibration calibration = new QuestCalibration();

  private Pose2d robotResetPose = new Pose2d();

  public Quest(QuestIO io) {
    this.io = io;
    setRobotResetPose(Pose2d.kZero);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    disconnectedAlert.set(!inputs.connected);
    lowBatteryAlert.set(inputs.batteryLevel < 25 && inputs.connected);

    if (DriverStation.isEnabled() && Constants.currentMode == Constants.Mode.REAL) {
      Pose2d robotPoseFromQuest = this.getRobotToField();
      
      // Only enable this when we know we're ready
      // BobotState.offerQuestMeasurement(
      //     new TimestampedPose(robotPoseFromQuest, inputs.timestamp));

      BobotState.updateQuestPose(robotPoseFromQuest);
    }
  }

  /**
   * The Oculus tracks relative to where it started, so we need to tell the Quest where the robot
   * started on the field.
   *
   * @param newRobotResetPose
   */
  public void setRobotResetPose(Pose2d newRobotResetPose) {
    this.robotResetPose = newRobotResetPose;
  }

  /**
   * Compares the current position of the headset to where it was last reset,
   * then transforms that by robot-to-quest and the known reset point on
   * the field.
   *
   * @return Quest to Field
   */
  @AutoLogOutput
  public Pose2d getQuestToField() {
    Transform2d poseRelativeToReset = inputs.rawPose.minus(inputs.resetRobotPose);
    return this.robotResetPose
        .transformBy(QuestConstants.robotToQuestTransform)
        .transformBy(poseRelativeToReset);
  }

  /**
   * Using the known position of the Quest on the field, get the pose of the Robot relative to the
   * Quest.
   *
   * @return Robot to Field
   */
  @AutoLogOutput
  public Pose2d getRobotToField() {
    return this.getQuestToField().transformBy(QuestConstants.robotToQuestTransform.inverse());
  }

  @Override
  public void simulationPeriodic() {}

  public Command calibrateCommand(Drive drive) {
    return calibration.determineOffsetToRobotCenter(
        drive, () -> inputs.pose, () -> inputs.questPose);
  }
}
