package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSystem;

public class ClimbMoveLeftArmCommand extends Command {
    private final ClimbSystem climbSystem;

    public ClimbMoveLeftArmCommand(ClimbSystem climbSystem){
        this.climbSystem = climbSystem;
        addRequirements(climbSystem);
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {

    }
}
