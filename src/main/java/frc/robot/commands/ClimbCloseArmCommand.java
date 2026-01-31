package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystem;

public class ClimbCloseArmCommand extends Command {
    private final ClimbSystem climbSystem;

    public ClimbCloseArmCommand(ClimbSystem climbSystem){
        this.climbSystem = climbSystem;
        addRequirements(climbSystem);
    }


    @Override
    public void initialize() {
        climbSystem.moveMotor(RobotMap.CLIMB_MOTOR_BACKWARD_SPEED);
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
        return climbSystem.isBottomSwitchPressed();
    }
}
