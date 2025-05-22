package frc.robot.subsystems.rollers.follow;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.feedforward_controller.FeedforwardController;

public class FollowRollersIOSim implements FollowRollersIO {
  private final DCMotorSim leader;
  private final DCMotorSim follower;

  private final boolean invertFollower;

  private final ProfiledPIDController controller;

  private final FeedforwardController ff;

  private boolean isClosedLoop = false;

  public FollowRollersIOSim(
      DCMotor leaderModel,
      DCMotor followerModel,
      double reduction,
      double moi,
      boolean invertFollower,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig,
      FeedforwardController ff) {
    this.ff = ff;
    leader =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(leaderModel, moi, reduction), leaderModel);
    follower =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(followerModel, moi, reduction), followerModel);

    this.controller =
        new ProfiledPIDController(
            gains.kP,
            gains.kI,
            gains.kD,
            new TrapezoidProfile.Constraints(
                mmConfig.MotionMagicCruiseVelocity, mmConfig.MotionMagicAcceleration));

    this.invertFollower = invertFollower;
  }

  @Override
  public void updateInputs(FollowRollersMagicIOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      stop();
    } else if (isClosedLoop) {
      double feedforward = FeedforwardController.calculate(ff, controller.getSetpoint());
      double feedback = controller.calculate(leader.getAngularPositionRotations());
      setVolts(feedforward + feedback);
    }

    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    leader.update(Constants.loopPeriodSecs);

    inputs.leaderPositionRotations = leader.getAngularPositionRotations();
    inputs.leaderVelocityRotationsPerSec = leader.getAngularVelocityRPM() / 60.0;

    inputs.leaderAppliedVoltage = leader.getInputVoltage();
    inputs.leaderSupplyCurrentAmps = leader.getCurrentDrawAmps();

    inputs.followerPositionRotations = leader.getAngularPositionRotations();
    inputs.followerVelocityRotationsPerSec = leader.getAngularVelocityRPM() / 60.0;

    inputs.followerAppliedVoltage = follower.getInputVoltage();
    inputs.followerSupplyCurrentAmps = leader.getCurrentDrawAmps();

    inputs.positionGoalRotations = controller.getGoal().position;
    inputs.positionSetpointRotations = controller.getSetpoint().position;
    inputs.velocitySetpointRotationsPerSec = controller.getSetpoint().velocity;
  }

  private void setVolts(double volts) {
    double voltage = MathUtil.clamp(volts, -12.0, 12.0);
    leader.setInputVoltage(voltage);
    follower.setInputVoltage(invertFollower ? -voltage : voltage);
  }

  @Override
  public void runVolts(double volts) {
    isClosedLoop = false;
    setVolts(volts);
  }

  @Override
  public void setGoal(double positionRotations) {
    controller.setGoal(new TrapezoidProfile.State(positionRotations, 0));
    isClosedLoop = true;
  }

  @Override
  public void resetPosition(double positionRotations) {
    leader.setAngle(Units.rotationsToRadians(positionRotations));
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
