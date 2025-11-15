package frc.robot.subsystems.rollers.single;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SingleRollerIOTalonFXSim extends SingleRollerIOTalonFX {
  private DCMotorSim motorSim;

  public SingleRollerIOTalonFXSim(
      int canId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig,
      DCMotor dcMotor) {
    this(
        canId,
        reduction,
        currentLimitAmps,
        invert,
        isBrakeMode,
        foc,
        gains,
        mmConfig,
        dcMotor,
        Rotation2d.fromRotations(Double.MIN_VALUE),
        Rotation2d.fromRotations(Double.MAX_VALUE));
  }

  public SingleRollerIOTalonFXSim(
      int canId,
      double reduction,
      double currentLimitAmps,
      boolean invert,
      boolean isBrakeMode,
      boolean foc,
      Slot0Configs gains,
      MotionMagicConfigs mmConfig,
      DCMotor dcMotor,
      Rotation2d minimumPosition,
      Rotation2d maximumPosition) {
    super(
        canId,
        reduction,
        currentLimitAmps,
        invert,
        isBrakeMode,
        foc,
        gains,
        mmConfig,
        minimumPosition,
        maximumPosition);

    motorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(dcMotor, 0.00002, reduction), dcMotor);
  }

  @Override
  public void setGoal(double positionRotations) {
    positionGoalRotations = positionRotations * reduction;
    talon.setControl(
        positionVoltage.withPosition(
            MathUtil.clamp(
                positionGoalRotations,
                minimumPosition.getRotations(),
                maximumPosition.getRotations())));
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    // Still do everything as expected for TalonFX
    super.updateInputs(inputs);

    // Update motor output from sim state
    motorSim.setInput(talon.getSimState().getMotorVoltage());
    motorSim.update(Constants.loopPeriodSecs);

    // (WIP) Take DCMotorSim configs and set them to TalonFX Sim
    talon.getSimState().setRawRotorPosition(motorSim.getAngularPositionRotations() * reduction);
    talon
        .getSimState()
        .setRotorVelocity(Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec() * reduction));
  }
}
