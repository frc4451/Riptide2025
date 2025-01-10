package frc.robot.subsystems.rollers.single;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SingleRollerIOSim implements SingleRollerIO {
  private final DCMotorSim sim;

  private double appliedVoltage = 0.0;

  public SingleRollerIOSim(DCMotor motorModel, double reduction, double moi) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      runVolts(0.0);
    }

    inputs.connected = true;
    sim.update(Constants.loopPeriodSecs);

    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void stop() {
    runVolts(0.0);
  }
}
