package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    public TalonFX motor = new TalonFX(23);
    public BeamBreak beamBreak = new BeamBreak(67);// TODO pick the right number also read jibjib.io

    public IntakeSubsystem() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void runMotor() {
        motor.set(IntakeConstants.kIntakeShootSpeed);
    }

    public void stopMotor() {
        motor.set(0);
    }
}
