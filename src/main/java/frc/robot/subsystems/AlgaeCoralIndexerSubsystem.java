// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeCoralIndexerConstants;
import frc.robot.Constants.AlgaeCoralIndexerConstants;

public class AlgaeCoralIndexerSubsystem extends SubsystemBase {
  /** Creates a new AlgaeCoralIndexerSubsystem. */
  private final TalonFX m_motor;
  private final DigitalInput m_algaebeambreak ;
  private final DigitalInput m_coralbeambreak ;
  /*instsmth motors and beam breaks */
  public AlgaeCoralIndexerSubsystem() {
    m_motor = new TalonFX(AlgaeCoralIndexerConstants.ALGAE_CORAL_MOTOR_ID) ; 
    /*kitkats r yummy */
    m_algaebeambreak = new DigitalInput(AlgaeCoralIndexerConstants.CORAL_BEAM_BREAK_ID) ;
    /*idk what to writ here  */
    m_coralbeambreak = new DigitalInput(AlgaeCoralIndexerConstants.ALGAE_BEAM_BREAK_ID) ;
    /*algae_beam_break_id is secretly James Bond */
    
  }


  public boolean hasAlgae (){
    return !m_algaebeambreak.get();
  }
  public boolean hasCoral () {
    return !m_coralbeambreak.get() ;
  }
  /*check if bakon has not stolen coral */
/*hi */
  public void voltagecontrol(double voltorb){
  m_motor.setVoltage(voltorb);
  
/*... */

  }
  public Voltage getIndexerVoltage() {
    return m_motor.getMotorVoltage().getValue();
}


  
    @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
