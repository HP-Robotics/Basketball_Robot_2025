// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  // this is not the right device ID.
  public TalonFX m_shooterLeaderMotor = new TalonFX(ShooterConstants.shooterLeaderMotorId);
  public TalonFX m_shooterFollowerMotor = new TalonFX(ShooterConstants.shooterFollowerMotorId);
  private boolean m_isRunning = false;

  public ShooterSubsystem() {
    m_shooterLeaderMotor.getConfigurator().apply(new TalonFXConfiguration());
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = ShooterConstants.shooterkP;
    slot0Configs.kI = ShooterConstants.shooterkI;
    slot0Configs.kD = ShooterConstants.shooterkD;
    slot0Configs.kS = ShooterConstants.shooterkS;
    slot0Configs.kV = ShooterConstants.shooterkV;
    m_shooterLeaderMotor.getConfigurator().apply(slot0Configs);
    m_shooterLeaderMotor.setNeutralMode(NeutralModeValue.Coast);

    m_shooterFollowerMotor.getConfigurator().apply(new TalonFXConfiguration());
    m_shooterFollowerMotor.setNeutralMode(NeutralModeValue.Coast);
    m_shooterFollowerMotor.setControl(new Follower(ShooterConstants.shooterLeaderMotorId, false));
  }

  public void runShooter() {
    m_shooterLeaderMotor.setControl(new VelocityDutyCycle(ShooterConstants.shooterSpeed));
    m_isRunning = true;

  }

  public void stopShooter() {
    m_shooterLeaderMotor.set(0);
    m_isRunning = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean atSpeed() {
    // TODO test this later
    return Math
        .abs(this.m_shooterLeaderMotor.getClosedLoopError().getValueAsDouble()) <= ShooterConstants.shooterAllowedError
        && m_isRunning;
  }
}