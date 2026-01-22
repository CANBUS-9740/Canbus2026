package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystem;

public class ClimbCloseLeftArmCommand extends Command {
    private final ClimbSystem climbSystem;

    public ClimbCloseLeftArmCommand(ClimbSystem climbSystem){
        this.climbSystem = climbSystem;
        addRequirements(climbSystem);
    }


    @Override
    public void initialize() {
        climbSystem.moveLeftMotor(RobotMap.LEFT_MOTOR_BACKWARD_SPEED);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        climbSystem.stopLeft();
    }

    @Override
    public boolean isFinished() {
        return climbSystem.isSwitch1Pressed();
    }
}
