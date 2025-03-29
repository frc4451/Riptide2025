package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
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

  private Pose2d fieldToRobotOrigin = Pose2d.kZero;

  public Quest(QuestIO io) {
    this.io = io;
    resetRobotPose(Pose2d.kZero);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    disconnectedAlert.set(!inputs.connected);
    lowBatteryAlert.set(inputs.connected && inputs.batteryLevel < 25);

    Pose2d fieldToRobot = getFieldToRobot();

    // Only enable this when we know we're ready
    // if (DriverStation.isEnabled() && Constants.currentMode == Constants.Mode.REAL) {
    //   BobotState.offerQuestMeasurement(
    //       new TimestampedPose(robotPoseFromQuest, inputs.timestamp));
    // }

    // Do this always for now just to confirm our transforms are correct.
    // Or, you may want to always track rotation. Do science.
    BobotState.updateQuestPose(fieldToRobot);
  }

  @Override
  public void simulationPeriodic() {}

  /**
   * The Oculus tracks relative to where it started, so we need to tell the Quest where the robot
   * started on the field.
   *
   * @param robotResetPose
   */
  public void resetRobotPose(Pose2d robotResetPose) {
    this.fieldToRobotOrigin = robotResetPose;
    io.zeroPosition();
    io.zeroHeading();
  }

  /**
   * Compares the current position of the headset to where it was last reset, then transforms that
   * by robot-to-quest and the known reset point on the field.
   *
   * @return Field to Quest
   */
  @AutoLogOutput
  public Pose2d getFieldToQuest() {
    return fieldToRobotOrigin
        .transformBy(QuestConstants.robotToQuestTransform)
        .transformBy(inputs.uncorrectedResetToQuest);
  }

  /**
   * Using the known position of the Quest on the field, get the pose of the Robot relative to the
   * Quest.
   *
   * @return Robot to Field
   */
  @AutoLogOutput
  public Pose2d getFieldToRobot() {
    return getFieldToQuest().transformBy(QuestConstants.robotToQuestTransform.inverse());
  }

  public Command calibrateCommand(Drive drive) {
    return calibration.determineOffsetToRobotCenter(
        drive,
        () ->
            new Pose2d(
                inputs.uncorrectedResetToQuest.getTranslation(),
                inputs.uncorrectedResetToQuest.getRotation()));
  }
}
