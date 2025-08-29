// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeCoralIndexerConstants;

public class AlgaeCoralIndexerSubsystem extends SubsystemBase {
  /** Creates a new AlgaeCoralIndexerSubsystem. */
  private final TalonFX m_motor;
  private final DigitalInput m_kewlbeambreak ;
  private final DigitalInput m_kewlbeambreak2 ;
  /*instsmth motors and beam breaks */
  public AlgaeCoralIndexerSubsystem() {
    m_motor = new TalonFX(AlgaeCoralIndexerConstants.kitkat) ; 
    /*kitkats r yummy */
    m_kewlbeambreak = new DigitalInput(AlgaeCoralIndexerConstants.canoe) ;
    /*idk what to wrie here  */
    m_kewlbeambreak2 = new DigitalInput(AlgaeCoralIndexerConstants.spiderman) ;
    /*spiderman is secretly James Bond */
    
  }


  private boolean hasAlgae (){
    return !m_kewlbeambreak.get();
  }
  private boolean hasCoral () {
    return !m_kewlbeambreak2.get() ;
  }
  /*check if bakon has not stolen coral */
/*hi */
  private void voltagecontrol(double voltorb){
  m_motor.setVoltage(voltorb);
  


  }
  public double getIndexerVoltage() {
    return m_motor.getMotorVoltage().getValue();
}
  public void algaeAndCoralOrNo() {
    if (hasCoral()) {
      if (hasAlgae()) {
        voltagecontrol(AlgaeCoralIndexerConstants.algaeRemovalVoltage);
      } else {
        voltagecontrol(0);
      }
    } else {
      voltagecontrol(AlgaeCoralIndexerConstants.coralReleaseVoltage); }
    }

  
    @Override
  public void periodic() {
    algaeAndCoralOrNo();
    // This method will be called once per scheduler run
  }
}
