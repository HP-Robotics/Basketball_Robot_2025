package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeShootCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;

    public IntakeShootCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        m_intakeSubsystem.motor.set(IntakeConstants.kIntakeShootSpeed);
    }

    @Override
    public boolean isFinished() {
        if (m_intakeSubsystem.beamBreak.beamBroken()) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.motor.set(0);
    }
}
