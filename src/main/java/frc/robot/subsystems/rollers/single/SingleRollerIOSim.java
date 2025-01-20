package frc.robot.subsystems.rollers.single;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class SingleRollerIOSim implements SingleRollerIO {
  private final DCMotorSim sim;

  private final PIDController controller;

  private double appliedVoltage = 0.0;

  private boolean closedLoop = false;

  public SingleRollerIOSim(
      DCMotor motorModel, double reduction, double moi, PIDConstants pidConstants) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
    controller = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    // DO NOT RUN WHEN DISABLED
    if (DriverStation.isDisabled()) {
      stop();
    } else if (closedLoop) {
      runVolts(controller.calculate(sim.getAngularPositionRad()));
    }

    inputs.connected = true;
    sim.update(Constants.loopPeriodSecs);

    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();

    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
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
