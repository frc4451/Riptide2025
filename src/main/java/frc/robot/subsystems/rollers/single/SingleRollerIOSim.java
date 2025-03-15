package frc.robot.subsystems.rollers.single;

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

public class SingleRollerIOSim implements SingleRollerIO {
  private final DCMotorSim sim;

  private final ProfiledPIDController controller;

  private final FeedforwardController ff;

  private boolean isClosedLoop = false;

  public SingleRollerIOSim(
      DCMotor motorModel,
      double reduction,
      double moi,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig,
      FeedforwardController ff) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);

    this.ff = ff;
    this.controller =
        new ProfiledPIDController(
            gains.kP,
            gains.kI,
            gains.kD,
            new TrapezoidProfile.Constraints(
                mmConfig.MotionMagicCruiseVelocity, mmConfig.MotionMagicAcceleration));
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      stop();
    } else if (isClosedLoop) {
      double feedforward = FeedforwardController.calculate(ff, controller.getSetpoint());
      double feedback = controller.calculate(sim.getAngularPositionRotations());
      setVolts(feedforward + feedback);
    }

    inputs.connected = true;
    sim.update(Constants.loopPeriodSecs);

    inputs.positionRotations = sim.getAngularPositionRotations();
    inputs.velocityRotationsPerSec = sim.getAngularVelocityRPM() / 60.0;

    inputs.appliedVoltage = sim.getInputVoltage();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();

    inputs.positionGoalRotations = controller.getGoal().position;
    inputs.positionSetpointRotations = controller.getSetpoint().position;
    inputs.velocitySetpointRotationsPerSec = controller.getSetpoint().velocity;
  }

  private void setVolts(double volts) {
    double voltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(voltage);
  }

  @Override
  public void runVolts(double volts) {
    isClosedLoop = false;
    setVolts(volts);
  }

  @Override
  public void setGoal(double positionRotations) {
    isClosedLoop = true;
    controller.setGoal(new TrapezoidProfile.State(positionRotations, 0));
  }

  @Override
  public void resetPosition(double positionRotations) {
    sim.setAngle(Units.rotationsToRadians(positionRotations));
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
