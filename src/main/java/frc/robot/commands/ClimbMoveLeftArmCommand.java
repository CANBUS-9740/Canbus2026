package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystem;

public class ClimbMoveLeftArmCommand extends Command {
    private final ClimbSystem climbSystem;

    public ClimbMoveLeftArmCommand(ClimbSystem climbSystem){
        this.climbSystem = climbSystem;
        addRequirements(climbSystem);
    }


    @Override
    public void initialize() {
        climbSystem.moveMotor1(RobotMap.MOTOR1_SPEED);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        climbSystem.stop1();
    }

    @Override
    public boolean isFinished() {
        return climbSystem.isSwitch1Pressed();
    }
}
