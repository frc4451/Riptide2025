package frc.robot.subsystems.rollers.single;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SingleRollerIOSim implements SingleRollerIO {
  private final DCMotorSim sim;

  public SingleRollerIOSim(DCMotor motorModel, double reduction, double moi) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      stop();
    }

    inputs.connected = true;
    sim.update(Constants.loopPeriodSecs);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();

    inputs.appliedVoltage = sim.getInputVoltage();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    sim.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  @Override
  public void resetPosition(double positionRad) {
    sim.setAngle(positionRad);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
