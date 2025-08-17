package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelIndexerSubsystem;
import frc.robot.Constants;


public class FunnelIntakeCommand extends Command {
    
    private final FunnelIndexerSubsystem m_funnel;
  
    public FunnelIntakeCommand(FunnelIndexerSubsystem subsystem) {
      this.m_funnel = subsystem;
      addRequirements(subsystem);
    }
  
    @Override
    public void execute() {
      boolean firstBroken = m_funnel.isFirstBeamBroken();
      boolean secondBroken = m_funnel.isSecondBeamBroken();
  
      if (!firstBroken && !secondBroken) {
        m_funnel.setMotorSpeed(Constants.FunnelIndexerConstants.m_fullSpeed);
      } else if (firstBroken && !secondBroken) {
        m_funnel.setMotorSpeed(Constants.FunnelIndexerConstants.m_halfSpeed);
      } else if (!firstBroken && secondBroken) {
        m_funnel.setMotorSpeed(Constants.FunnelIndexerConstants.m_reverseSpeed);
      } else {
        m_funnel.setMotorSpeed(Constants.FunnelIndexerConstants.m_stopSpeed);
      }
    }
  
    @Override
    public void end(boolean interrupted) {
      m_funnel.stopFunnel();
    }
  }
  