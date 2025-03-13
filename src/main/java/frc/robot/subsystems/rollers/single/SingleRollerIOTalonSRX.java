package frc.robot.subsystems.rollers.single;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class SingleRollerIOTalonSRX implements SingleRollerIO {
  private final TalonSRX talon;

  public SingleRollerIOTalonSRX(int canId, int currentLimitAmps, boolean invert) {
    talon = new TalonSRX(canId);

    talon.setInverted(invert);
    talon.setNeutralMode(NeutralMode.Brake);

    talon.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(
            true,
            currentLimitAmps,
            currentLimitAmps,
            0.04)); // 0.04 s is the same as the Phoenix 6 default of 40 ms

    talon.configVoltageCompSaturation(12.0);
    talon.enableVoltageCompensation(true);
  }

  @Override
  public void updateInputs(SingleRollerIOInputs inputs) {
    inputs.connected = talon.getLastError() == ErrorCode.OK;

    inputs.positionRad = Double.NaN;
    inputs.velocityRadPerSec = Double.NaN;

    inputs.appliedVoltage = talon.getMotorOutputVoltage();
    inputs.supplyCurrentAmps = talon.getSupplyCurrent();
    inputs.torqueCurrentAmps = talon.getStatorCurrent();
    inputs.temperatureCelsius = talon.getTemperature();
  }

  @Override
  public void runVolts(double volts) {
    talon.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
  }

  @Override
  public void resetPosition(double positionRad) {
    throw new IllegalStateException("There is no position for a TalonSRX");
  }

  @Override
  public void stop() {
    talon.set(TalonSRXControlMode.Disabled, 0);
  }
}
