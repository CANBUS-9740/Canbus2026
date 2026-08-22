package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StaticShooterSystem;


public class ShooterFeederBackwardsCommand extends Command {

    private final StaticShooterSystem staticShooterSystem;

    public ShooterFeederBackwardsCommand(StaticShooterSystem staticShooterSystem){
        this.staticShooterSystem = staticShooterSystem;
        addRequirements(staticShooterSystem);
    }


    @Override
    public void initialize() {
        staticShooterSystem.ShooterFeederBackwards();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        staticShooterSystem.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
