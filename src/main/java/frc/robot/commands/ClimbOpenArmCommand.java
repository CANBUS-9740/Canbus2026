package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSystem;

public class ClimbOpenArmCommand extends Command {
    private final ClimbSystem climbSystem;
    private final double targetPositionMeters;

    public ClimbOpenArmCommand(ClimbSystem climbSystem, double targetPositionMeters){
        this.climbSystem = climbSystem;
        this.targetPositionMeters = targetPositionMeters;
        addRequirements(climbSystem);
    }


    @Override
    public void initialize() {
        climbSystem.setTargetPosition(targetPositionMeters);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

        climbSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
