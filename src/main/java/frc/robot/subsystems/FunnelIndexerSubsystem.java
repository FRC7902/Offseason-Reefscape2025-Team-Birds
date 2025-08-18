// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FunnelIndexerConstants;;

public class FunnelIndexerSubsystem extends SubsystemBase {
  private final TalonFX funnelMotor = new TalonFX(FunnelIndexerConstants.kFunnelMotorID);
  private final DigitalInput shallowBeamBreak = new DigitalInput(FunnelIndexerConstants.kShallowBeamBreakPort); //Shallow Beam Break
  private final DigitalInput deepBeamBreak = new DigitalInput(FunnelIndexerConstants.kDeepBeamBreakPort); // Deep Beam Break

  public FunnelIndexerSubsystem() {}

  public void setMotorSpeed(double speed) {
    funnelMotor.set(speed);
  }

  public boolean isShallowBeamBroken() {
    return !shallowBeamBreak.get(); 
  }

  public boolean isDeepBeamBroken() {
    return !deepBeamBreak.get(); 
  }

  public boolean isCoralPresent() {
    return isShallowBeamBroken() || isDeepBeamBroken();
  }

  public void stopFunnel() {
    funnelMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("First Beam Broken", isShallowBeamBroken());
    SmartDashboard.putBoolean("Second Beam Broken", isDeepBeamBroken());
  }
}
