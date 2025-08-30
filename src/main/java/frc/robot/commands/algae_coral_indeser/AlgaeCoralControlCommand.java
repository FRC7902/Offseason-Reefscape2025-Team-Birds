package frc.robot.commands.algae_coral_indeser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeCoralIndexerConstants;
import frc.robot.RobotContainer;

public class AlgaeCoralControlCommand extends Command {
    public AlgaeCoralControlCommand() {
        addRequirements(RobotContainer.m_AlgaeCoralIndexerSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (RobotContainer.m_AlgaeCoralIndexerSubsystem.hasCoral()) {
            if (RobotContainer.m_AlgaeCoralIndexerSubsystem.hasAlgae()) {
                RobotContainer.m_AlgaeCoralIndexerSubsystem
                        .voltagecontrol(AlgaeCoralIndexerConstants.algaeRemovalVoltage);
            } else {
                RobotContainer.m_AlgaeCoralIndexerSubsystem.voltagecontrol(0);
            }
        } else {
            RobotContainer.m_AlgaeCoralIndexerSubsystem.voltagecontrol(AlgaeCoralIndexerConstants.coralReleaseVoltage);
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
