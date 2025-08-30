// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeCoralIndexerConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;

public class AlgaeCoralIndexerSubsystem extends SubsystemBase {
  /** Creates a new AlgaeCoralIndexerSubsystem. */
  private final TalonFX m_motor;
  private final DigitalInput m_algaebeambreak;
  private final DigitalInput m_coralbeambreak;

  public AlgaeCoralIndexerSubsystem() {
    m_motor = new TalonFX(AlgaeCoralIndexerConstants.ALGAE_CORAL_MOTOR_ID);
    m_algaebeambreak = new DigitalInput(AlgaeCoralIndexerConstants.CORAL_BEAM_BREAK_ID);
    m_coralbeambreak = new DigitalInput(AlgaeCoralIndexerConstants.ALGAE_BEAM_BREAK_ID);

  }

  public boolean hasAlgae() {
    return !m_algaebeambreak.get();
  }

  public boolean hasCoral() {
    return !m_coralbeambreak.get();
  }

  public void voltagecontrol(double voltorb) {
    m_motor.setVoltage(voltorb);

    /* ... */

  }

  public Voltage getIndexerVoltage() {
    return m_motor.getMotorVoltage().getValue();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
