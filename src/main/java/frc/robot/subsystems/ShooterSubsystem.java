// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  // this is not the right device ID.
  public TalonFX m_shooterMotor1 = new TalonFX(41);
  public TalonFX m_shooterMotor2 = new TalonFX(42);// TODO

  public ShooterSubsystem() {
  }

  public void runShooter() {
    m_shooterMotor1.set(ShooterConstants.shooterSpeed);
    m_shooterMotor2.set(ShooterConstants.shooterSpeed);
  }

  public void stopShooter() {
    m_shooterMotor1.set(0);
    m_shooterMotor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean atSpeed() {
    if (this.m_shooterMotor1.getVelocity().getValueAsDouble() / 512.0 <= ShooterConstants.shooterSpeed
        && this.m_shooterMotor2.getVelocity().getValueAsDouble() / 512.0 <= ShooterConstants.shooterSpeed) {
      return true;
    } else {
      return false;
    }
  }
}