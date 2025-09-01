package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelIndexerSubsystem;
import frc.robot.Constants.FunnelIndexerConstants;

public class FunnelOuttakeCommand extends Command {

    private final FunnelIndexerSubsystem m_funnel;

    public FunnelOuttakeCommand(FunnelIndexerSubsystem subsystem) {
        this.m_funnel = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        m_funnel.setMotorSpeed(FunnelIndexerConstants.m_halfSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        m_funnel.stopFunnel();
    }

}