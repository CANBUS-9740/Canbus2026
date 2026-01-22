package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ClimbSystem;

public class ClimbOpenLeftAndRightArmCommand extends Command {
    private final ClimbSystem climbSystem;
    private final double targetPositionDegrees;

    public ClimbOpenLeftAndRightArmCommand(ClimbSystem climbSystem, double targetPositionDegrees){
        this.climbSystem = climbSystem;
        this.targetPositionDegrees = targetPositionDegrees;
        addRequirements(climbSystem);
    }


    @Override
    public void initialize() {
        climbSystem.moveMotor1(RobotMap.MOTOR1_SPEED);
        climbSystem.moveMotor2(RobotMap.MOTOR2_SPEED);
        climbSystem.setTargetPosition(targetPositionDegrees);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        climbSystem.stop1();
        climbSystem.stop2();
    }

    @Override
    public boolean isFinished() {
        return climbSystem.isSwitch1Pressed()&& climbSystem.isSwitch2Pressed();
    }
}
