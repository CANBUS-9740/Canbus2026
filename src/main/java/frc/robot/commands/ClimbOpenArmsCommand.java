package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystem;

public class ClimbOpenArmsCommand extends Command {

    private final ClimbSystem climbSystem;
    private final double targetPositionMeters;

    private boolean leftDone;
    private boolean rightDone;

    public ClimbOpenArmsCommand(ClimbSystem climbSystem, double targetPositionMeters) {
        this.climbSystem = climbSystem;
        this.targetPositionMeters = targetPositionMeters;

        addRequirements(climbSystem);
    }

    @Override
    public void initialize() {
        leftDone = false;
        rightDone = false;

        climbSystem.setTargetPosition(targetPositionMeters);
    }

    @Override
    public void execute() {
        if (!leftDone && climbSystem.isAtLeftTarget(targetPositionMeters)) {
            climbSystem.stopLeft();
            leftDone = true;
        }

        if (!rightDone && climbSystem.isAtRightTarget(targetPositionMeters)) {
            climbSystem.stopRight();
            rightDone = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbSystem.stopLeft();
        climbSystem.stopRight();
    }

    @Override
    public boolean isFinished() {
        return leftDone && rightDone;
    }
}
