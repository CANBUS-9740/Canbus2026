package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSystem;

public class ClimbOpenLeftAndRightArmCommand extends Command {
    private final ClimbSystem climbSystem;
    private final double targetPositionMeters;

    public ClimbOpenLeftAndRightArmCommand(ClimbSystem climbSystem, double targetPositionMeters){
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
        climbSystem.stopLeft();
        climbSystem.stopRight();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
