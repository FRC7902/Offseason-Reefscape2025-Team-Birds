package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelIndexerSubsystem;
import frc.robot.Constants.FunnelIndexerConstants;


public class FunnelIntakeCommand extends Command {
    
    private final FunnelIndexerSubsystem m_funnel;
  
    public FunnelIntakeCommand(FunnelIndexerSubsystem subsystem) {
      this.m_funnel = subsystem;
      addRequirements(subsystem);
    }
  
    @Override
    public void execute() {
      boolean shallowBroken = m_funnel.isShallowBeamBroken();
      boolean deepBroken = m_funnel.isDeepBeamBroken();
  
      if (!shallowBroken && !deepBroken) {
        m_funnel.setMotorSpeed(FunnelIndexerConstants.m_fullSpeed);
      } else if (shallowBroken && !deepBroken) {
        m_funnel.setMotorSpeed(FunnelIndexerConstants.m_halfSpeed);
      } else if (!shallowBroken && deepBroken) {
        m_funnel.setMotorSpeed(FunnelIndexerConstants.m_stopSpeed);
      } else {
        m_funnel.setMotorSpeed(FunnelIndexerConstants.m_reverseSpeed);
      }
    }
  
    @Override
    public void end(boolean interrupted) {
      m_funnel.stopFunnel();
    }
  }
  