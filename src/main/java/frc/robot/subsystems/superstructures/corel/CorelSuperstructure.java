package frc.robot.subsystems.superstructures.corel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructures.corel.conveyor.CorelConveyor;
import frc.robot.subsystems.superstructures.corel.elevator.CorelElevator;
import frc.robot.subsystems.superstructures.corel.elevator.CorelElevatorSetpoint;

public class CorelSuperstructure extends SubsystemBase {
    private final CorelElevator elevator;

    // If we go fixed angle, use this, otherwise make a pivot for this
    private final CorelConveyor conveyor;

    public CorelSuperstructure(CorelElevator corelElevator, CorelConveyor corelRoller) {
        corelElevator.setName("CorelSuperstructure/Elevator");
        corelRoller.setName("CorelSuperstructure/Conveyor");

        this.elevator = corelElevator;
        this.conveyor = corelRoller;
    }

    public Trigger isElevatorMoving() {
        return new Trigger(() -> this.elevator.isElevatorMoving());
    }

    public Command shootCorel() {
        return this.conveyor.runRoller(12);
    }

    public Command raiseElevatorToL1() {
        return this.elevator.setGoalInchesCommand(CorelElevatorSetpoint.L1.setpointInches);
    }

    public Command raiseElevatorToL2() {
        return this.elevator.setGoalInchesCommand(CorelElevatorSetpoint.L2.setpointInches);
    }

    public Command raiseElevatorToL3() {
        return this.elevator.setGoalInchesCommand(CorelElevatorSetpoint.L3.setpointInches);
    }

    public Command raiseElevatorToL4() {
        return this.elevator.setGoalInchesCommand(CorelElevatorSetpoint.L4.setpointInches);
    }
}
