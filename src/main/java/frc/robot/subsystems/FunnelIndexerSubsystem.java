// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelIndexerSubsystem extends SubsystemBase {

  private final TalonFX funnelMotor = new TalonFX(1); // CAN ID for motor
  private final DigitalInput firstBeamBreak = new DigitalInput(0); // DIO port 0
  private final DigitalInput secondBeamBreak = new DigitalInput(1); // DIO port 1

  /** Creates a new FunnelIndexerSubsystem. */
  public FunnelIndexerSubsystem() {}

  //Sets the speed of the Motor
  public void setMotorSpeed(double speed) {
    funnelMotor.set(speed);
  }

  //Checks if coral is present in either beam
  public boolean isCoralPresent() {
    return isFirstBeamBroken() || isSecondBeamBroken();
  }

  public boolean isFirstBeamBroken() {
    return !firstBeamBreak.get(); // Invert: beam broken = false from DigitalInput
  }

  public boolean isSecondBeamBroken() {
    return !secondBeamBreak.get();
  }

  //Stops the Funnel
  public void stopFunnel() {
    funnelMotor.stopMotor();
  }

  /** Updates motor speed based on beam break sensor states */
  public void updateFunnelPosition() {
    boolean firstBroken = isFirstBeamBroken();
    boolean secondBroken = isSecondBeamBroken();

    if (!firstBroken && !secondBroken) {
      // Both NOT broken: run forward full speed
      setMotorSpeed(Constants.AlgaeCoralIndexerConstants.m_fullSpeed);
    } else if (firstBroken && !secondBroken) {
      // Only first broken: run forward half speed
      setMotorSpeed(Constants.AlgaeCoralIndexerConstants.m_halfSpeed);
    } else if (!firstBroken && secondBroken) {
      // Only second broken: run reverse full speed
      setMotorSpeed(Constants.AlgaeCoralIndexerConstants.m_reverseSpeed);
    } else {
      // Both broken: stop
      setMotorSpeed(Constants.AlgaeCoralIndexerConstants.m_stopSpeed);
    }
  }

  @Override
  public void periodic() {
    
    // Output sensor states to SmartDashboard for testing
    SmartDashboard.putBoolean("First Beam Broken", isFirstBeamBroken());
    SmartDashboard.putBoolean("Second Beam Broken", isSecondBeamBroken());

    // Call update logic every cycle (or remove if you only want it in a command)
    updateFunnelPosition();
  }
}
