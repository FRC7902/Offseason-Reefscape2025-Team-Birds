// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class FunnelIndexerSubsystem extends SubsystemBase {
  private final TalonFX funnelMotor = new TalonFX(Constants.FunnelIndexerConstants.kFunnelMotorID);
  private final DigitalInput firstBeamBreak = new DigitalInput(Constants.FunnelIndexerConstants.kFirstBeamBreakPort);
  private final DigitalInput secondBeamBreak = new DigitalInput(Constants.FunnelIndexerConstants.kSecondBeamBreakPort);

  public FunnelIndexerSubsystem() {}

  public void setMotorSpeed(double speed) {
    funnelMotor.set(speed);
  }

  public boolean isFirstBeamBroken() {
    return !firstBeamBreak.get(); 
  }

  public boolean isSecondBeamBroken() {
    return !secondBeamBreak.get(); 
  }

  public boolean isCoralPresent() {
    return isFirstBeamBroken() || isSecondBeamBroken();
  }

  public void stopFunnel() {
    funnelMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("First Beam Broken", isFirstBeamBroken());
    SmartDashboard.putBoolean("Second Beam Broken", isSecondBeamBroken());
  }
}
