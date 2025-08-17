public class FunnelIntakeCommand extends CommandBase {
    private final FunnelIndexerSubsystem funnel;
  
    public FunnelIntakeCommand(FunnelIndexerSubsystem subsystem) {
      this.funnel = subsystem;
      addRequirements(subsystem);
    }
  
    @Override
    public void execute() {
      boolean firstBroken = funnel.isFirstBeamBroken();
      boolean secondBroken = funnel.isSecondBeamBroken();
  
      if (!firstBroken && !secondBroken) {
        funnel.setMotorSpeed(Constants.FunnelIndexerConstants.kFullSpeed);
      } else if (firstBroken && !secondBroken) {
        funnel.setMotorSpeed(Constants.FunnelIndexerConstants.kHalfSpeed);
      } else if (!firstBroken && secondBroken) {
        funnel.setMotorSpeed(Constants.FunnelIndexerConstants.kReverseSpeed);
      } else {
        funnel.setMotorSpeed(Constants.FunnelIndexerConstants.kStopSpeed);
      }
    }
  
    @Override
    public void end(boolean interrupted) {
      funnel.stopFunnel();
    }
  }
  