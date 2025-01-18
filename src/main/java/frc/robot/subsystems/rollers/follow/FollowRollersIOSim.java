package frc.robot.subsystems.rollers.follow;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FollowRollersIOSim implements FollowRollersIO {
  private final DCMotorSim leader;
  private final DCMotorSim follower;

  private final PIDController controller;

  private double appliedVoltage = 0.0;

  private boolean closedLoop = false;

  private final boolean invertFollower;

  public FollowRollersIOSim(
      DCMotor leaderModel,
      DCMotor followerModel,
      double reduction,
      double moi,
      PIDConstants pidConstants,
      boolean invertFollower) {
    leader =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(leaderModel, moi, reduction), leaderModel);
    follower =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(followerModel, moi, reduction), followerModel);

    controller = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);

    this.invertFollower = invertFollower;
  }

  @Override
  public void updateInputs(FollowRollersIOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      stop();
    } else if (closedLoop) {
      runVolts(controller.calculate(leader.getAngularPositionRad()));
    }

    inputs.connected = true;
    leader.update(Constants.loopPeriodSecs);

    inputs.leaderPositionRad = leader.getAngularPositionRad();
    inputs.leaderVelocityRadPerSec = leader.getAngularVelocityRadPerSec();

    inputs.leaderAppliedVoltage = appliedVoltage;
    inputs.leaderSupplyCurrentAmps = leader.getCurrentDrawAmps();

    inputs.followerPositionRad = leader.getAngularPositionRad();
    inputs.followerVelocityRadPerSec = leader.getAngularVelocityRadPerSec();

    inputs.followerAppliedVoltage = appliedVoltage;
    inputs.followerSupplyCurrentAmps = leader.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    leader.setInputVoltage(appliedVoltage);
    follower.setInputVoltage(invertFollower ? appliedVoltage : -appliedVoltage);
  }

  @Override
  public void runPosition(double positionRad) {
    closedLoop = true;
    controller.setSetpoint(positionRad);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
