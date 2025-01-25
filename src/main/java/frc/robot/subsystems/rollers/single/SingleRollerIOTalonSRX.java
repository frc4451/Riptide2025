package frc.robot.subsystems.rollers.single;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SingleRollerIOTalonSRX implements SingleRollerIO {
  private final TalonSRX talon;
  private final double reduction;

  private final double tempCelsius;
  private final double statorCurrentAmps;
  private final double supplyCurrentAmps;
  private final double voltage;

  public SingleRollerIOTalonSRX(int canID, double reduction, int currentLimitAmps, boolean invert) {
    talon = new TalonSRX(canID);
    this.reduction = reduction;

    tempCelsius = talon.getTemperature();
    statorCurrentAmps = talon.getStatorCurrent();
    supplyCurrentAmps = talon.getSupplyCurrent();
    voltage = talon.getBusVoltage();

    talon.setInverted(invert);
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configContinuousCurrentLimit(currentLimitAmps);
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    inputs.connected = talon.getLastError() == ErrorCode.OK;

    inputs.positionRad = Double.NaN;
    inputs.velocityRadPerSec = Double.NaN;

    inputs.appliedVoltage = talon.getBusVoltage();
    inputs.supplyCurrentAmps = talon.getSupplyCurrent();
    inputs.torqueCurrentAmps = talon.getStatorCurrent();
    inputs.temperatureCelsius = talon.getTemperature();
  }

  @Override
  public void runVolts(double volts) {
    talon.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
  }

  @Override
  public void runPosition(double position) {
    throw new IllegalStateException("There is no position control for a TalonSRX");
  }

  @Override
  public void stop() {
    talon.set(TalonSRXControlMode.Disabled, 0);
  }
}
