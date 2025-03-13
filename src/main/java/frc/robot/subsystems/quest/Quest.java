package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Quest extends VirtualSubsystem {
  private final QuestIO io;
  private final QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Quest Disconnected!", AlertType.kWarning);
  private final Alert lowBatteryAlert =
      new Alert("Quest Battery is Low! (<25%)", AlertType.kWarning);

  private final QuestCalibration calibration = new QuestCalibration();

  public Quest(QuestIO io) {
    this.io = io;
    resetPose(Pose2d.kZero);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Oculus", inputs);

    disconnectedAlert.set(!inputs.connected);
    lowBatteryAlert.set(inputs.batteryLevel < 25 && inputs.connected);

    // if (DriverStation.isEnabled() && Constants.currentMode == Constants.Mode.REAL) {
    //   BobotState.offerQuestMeasurement(new TimestampedPose(inputs.robotPose, inputs.timestamp));
    // }
  }

  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }

  @Override
  public void simulationPeriodic() {}

  public Command calibrateCommand(Drive drive) {
    return calibration.determineOffsetToRobotCenter(
        drive, () -> inputs.robotPose, () -> inputs.questPose);
  }
}
