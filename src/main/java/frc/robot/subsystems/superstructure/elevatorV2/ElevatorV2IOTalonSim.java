// package frc.robot.subsystems.superstructure.elevatorV2;

// import org.littletonrobotics.junction.Logger;

// import com.ctre.phoenix6.sim.TalonFXSimState;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.ElevatorSim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
// import frc.robot.Constants;

// public class ElevatorV2IOTalonSim extends ElevatorV2IOTalonFX {
//   private final ElevatorSim elevatorSim;

//   private final TalonFXSimState talonFXSimState;

//   public ElevatorV2IOTalonSim(
//       int leaderCanId,
//       int followerCanId,
//       double reduction,
//       double currentLimitAmps,
//       boolean invert,
//       boolean invertFollower,
//       boolean isBrakeMode,
//       boolean foc,
//       double inchesPerRad,
//       DCMotor motorSystem,
//       double carriageMassKg,
//       double minHeightMeters,
//       double maxHeightMeters,
//       boolean simulateGravity,
//       double startingHeight) {
//     super(
//         leaderCanId,
//         followerCanId,
//         reduction,
//         currentLimitAmps,
//         invert,
//         invertFollower,
//         isBrakeMode,
//         foc,
//         inchesPerRad);

//     elevatorSim =
//         new ElevatorSim(
//             motorSystem,
//             reduction,
//             carriageMassKg,
//             Units.inchesToMeters(inchesPerRad),
//             minHeightMeters,
//             maxHeightMeters,
//             simulateGravity,
//             startingHeight);

//     talonFXSimState = leader.getSimState();
//   }

//   @Override
//   public void updateInputs(ElevatorV2IOInputs inputs) {
//     super.updateInputs(inputs);

//     talonFXSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

//     elevatorSim.update(Constants.loopPeriodSecs);
//     elevatorSim.setInput(talonFXSimState.getMotorVoltage());

//     RoboRioSim.setVInVoltage(
//         BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

//     talonFXSimState.setRawRotorPosition(
//         elevatorSim.getPositionMeters() / ElevatorV2Constants.motorConversionFactor);
//     talonFXSimState.setRotorVelocity(
//         elevatorSim.getVelocityMetersPerSecond() / ElevatorV2Constants.motorConversionFactor);

//     Logger.recordOutput("ElevatorSim/PositionInches",
// Units.metersToInches(elevatorSim.getPositionMeters()));
//     Logger.recordOutput("ElevatorSim/VelocityMetersPerSec",
// elevatorSim.getVelocityMetersPerSecond());
//   }

//   @Override
//   public double getPositionInches() {
//     return Units.metersToInches(this.elevatorSim.getPositionMeters());
//   }
// }
