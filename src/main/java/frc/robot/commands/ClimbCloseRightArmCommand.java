package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystem;

public class ClimbCloseRightArmCommand extends Command {
    private final ClimbSystem climbSystem;

    public ClimbCloseRightArmCommand(ClimbSystem climbSystem){
        this.climbSystem = climbSystem;
        addRequirements(climbSystem);
    }


    @Override
    public void initialize() {
        climbSystem.moveRightMotor(RobotMap.CLIMB_RIGHT_MOTOR_BACKWARD_SPEED);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        climbSystem.stopRight();
    }

    @Override
    public boolean isFinished() {
        return climbSystem.isBottomSwitchRightPressed();
    }
}
