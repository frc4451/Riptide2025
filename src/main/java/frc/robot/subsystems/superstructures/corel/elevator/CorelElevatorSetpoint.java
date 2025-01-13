package frc.robot.subsystems.superstructures.corel.elevator;

public enum CorelElevatorSetpoint {
  // These values are totally wrong. They're just here for testing
  INITIAL(0.0),
  L1(10.0),
  L2(20.0),
  L3(30.0),
  L4(40.0);

  public double setpointInches;

  CorelElevatorSetpoint(double setpointInches) {
    this.setpointInches = setpointInches;
  }
}
