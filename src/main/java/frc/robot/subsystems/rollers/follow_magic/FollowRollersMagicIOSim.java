package frc.robot.subsystems.rollers.follow_magic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FollowRollersMagicIOSim implements FollowRollersMagicIO {
  private final DCMotorSim leader;
  private final DCMotorSim follower;

  private final boolean invertFollower;

  public FollowRollersMagicIOSim(
      DCMotor leaderModel,
      DCMotor followerModel,
      double reduction,
      double moi,
      boolean invertFollower) {
    leader =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(leaderModel, moi, reduction), leaderModel);
    follower =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(followerModel, moi, reduction), followerModel);

    this.invertFollower = invertFollower;
  }

  @Override
  public void updateInputs(FollowRollersIOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      stop();
    }

    inputs.connected = true;
    leader.update(Constants.loopPeriodSecs);

    inputs.leaderPositionRotations = leader.getAngularPositionRad();
    inputs.leaderVelocityRotationsPerSec = leader.getAngularVelocityRadPerSec();

    inputs.leaderAppliedVoltage = leader.getInputVoltage();
    inputs.leaderSupplyCurrentAmps = leader.getCurrentDrawAmps();

    inputs.followerPositionRotations = leader.getAngularPositionRad();
    inputs.followerVelocityRotationsPerSec = leader.getAngularVelocityRadPerSec();

    inputs.followerAppliedVoltage = follower.getInputVoltage();
    inputs.followerSupplyCurrentAmps = leader.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    double voltage = MathUtil.clamp(volts, -12.0, 12.0);
    leader.setInputVoltage(voltage);
    follower.setInputVoltage(invertFollower ? -voltage : voltage);
  }

  @Override
  public void resetPosition(double positionRad) {
    leader.setAngle(positionRad);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
