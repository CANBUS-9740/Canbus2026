package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystem;

public class ClimbCloseArmsCommand extends Command {

    private final ClimbSystem climbSystem;

    private boolean leftDone;
    private boolean rightDone;

    public ClimbCloseArmsCommand(ClimbSystem climbSystem){
        this.climbSystem = climbSystem;
        addRequirements(climbSystem);
    }

    @Override
    public void initialize() {
        leftDone = false;
        rightDone = false;

        climbSystem.setMotors(RobotMap.CLIMB_MOTOR_BACKWARD_SPEED);
    }

    @Override
    public void execute() {
        if (!leftDone && climbSystem.isBottomSwitchLeftPressed()) {
            climbSystem.stopLeft();
            leftDone = true;
        }

        if (!rightDone && climbSystem.isBottomSwitchRightPressed()) {
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
