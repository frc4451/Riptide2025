package frc.robot.subsystems.superstructure.shooter;

import frc.robot.subsystems.rollers.single.SingleRoller;
import frc.robot.subsystems.rollers.single.SingleRollerIO;
import frc.robot.subsystems.superstructure.can_range.CanRange;
import frc.robot.subsystems.superstructure.can_range.CanRangeIO;
import frc.robot.subsystems.superstructure.modes.SuperStructureModes;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SingleRoller {
  private final CanRange coralSensor;

  private ShooterModes currentShooterMode = ShooterModes.NONE;

  public Shooter(String name, SingleRollerIO io, CanRangeIO coralSensorIO) {
    super(name, io);
    coralSensor = new CanRange(name + "/CoralSensor", coralSensorIO);
  }

  public void periodic(SuperStructureModes superMode) {
    super.periodic();
    coralSensor.periodic();

    switch (currentShooterMode) {
      case INTAKE:
        if (!coralSensor.isDetected()) {
          io.runVolts(currentShooterMode.voltage);
        } else {
          io.stop();
        }
        break;
      case SHOOT, L4_SHOOT:
        if (superMode == SuperStructureModes.L4Coral) {
          currentShooterMode = ShooterModes.L4_SHOOT;
        }
        io.runVolts(currentShooterMode.voltage);
        break;
      case ALGAE_INTAKED, ALGAE_SHOOT:
        io.runVolts(currentShooterMode.voltage);
        break;
      case ALGAE_INTAKING:
        io.runVolts(currentShooterMode.voltage);
        if (inputs.supplyCurrentAmps >= 10) {
          currentShooterMode = ShooterModes.ALGAE_INTAKED;
        }
        break;
      default:
      case NONE:
        io.stop();
        break;
    }

    Logger.recordOutput(name + "/ShooterMode", currentShooterMode);
  }

  public void setShooterMode(ShooterModes nextShooterMode) {
    if (currentShooterMode != nextShooterMode) {
      currentShooterMode = nextShooterMode;
    }
  }

  public boolean isCoralDetected() {
    return coralSensor.isDetected();
  }
}
