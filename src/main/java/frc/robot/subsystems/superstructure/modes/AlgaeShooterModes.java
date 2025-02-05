package frc.robot.subsystems.superstructure.modes;

public enum AlgaeShooterModes {
    NONE(0.0),
    INTAKE(-3.0),
    SHOOT(3.0),
    ;
    
    public final double voltage;

    private AlgaeShooterModes(double voltage){
        this.voltage = voltage;
    }
}
