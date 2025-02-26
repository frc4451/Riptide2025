package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Quest extends VirtualSubsystem {
  private final QuestIO io;
  private final QuestIOInputsAutoLogged inputs = new QuestIOInputsAutoLogged();

  private final Alert disconnectedAlert = new Alert("Quest Disconnected!", AlertType.kWarning);
  private final Alert lowBatteryAlert =
      new Alert("Quest Battery is Low! (<25%)", AlertType.kWarning);

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

    if (DriverStation.isEnabled()
        && inputs.connected
        && Constants.currentMode == Constants.Mode.REAL) {
      BobotState.offerQuestMeasurement(new TimestampedPose(inputs.robotPose, inputs.timestamp));
    }
  }

  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }

  @Override
  public void simulationPeriodic() {}

  // -- Calculate Quest Offset (copied from
  // https://github.com/FRC5010/Reefscape2025/blob/main/TigerShark2025/src/main/java/org/frc5010/common/sensors/camera/QuestNav.java#L65) --

  private Translation2d calculatedOffsetToRobot = new Translation2d();
  private double calculateOffsetCount = 1;

  private Translation2d calculateOffsetToRobot() {
    Pose2d currentPose = inputs.robotPose;

    Rotation2d angle = currentPose.getRotation();
    Translation2d displacement = currentPose.getTranslation();

    double x =
        ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
            / (2 * (1 - angle.getCos()));
    double y =
        ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
            / (2 * (1 - angle.getCos()));

    return new Translation2d(x, y);
  }

  public Command determineOffsetToRobotCenter(Drive drive) {
    return Commands.repeatingSequence(
        Commands.run(
                () -> {
                  drive.runVelocity(new ChassisSpeeds(0, 0, 0.3141));
                },
                drive)
            .withTimeout(0.5),
        Commands.runOnce(
                () -> {
                  // Update current offset
                  Translation2d offset = calculateOffsetToRobot();

                  calculatedOffsetToRobot =
                      calculatedOffsetToRobot
                          .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                          .plus(offset.div(calculateOffsetCount + 1));
                  calculateOffsetCount++;

                  SmartDashboard.putNumberArray(
                      "Quest Calculated Offset to Robot Center",
                      new double[] {
                        calculatedOffsetToRobot.getX(), calculatedOffsetToRobot.getY()
                      });
                })
            .onlyIf(() -> inputs.questPose.getRotation().getDegrees() > 30));
  }
}
