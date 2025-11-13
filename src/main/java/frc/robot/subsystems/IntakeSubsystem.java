package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BeamBreak;

public class IntakeSubsystem extends SubsystemBase {
    public TalonFX motor = new TalonFX(23);
    BeamBreak beamBreak = new BeamBreak(67);// TODO pick the right number also read jibjib.io

    public IntakeSubsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
