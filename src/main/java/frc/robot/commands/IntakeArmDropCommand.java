package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeArmSystem;

public class IntakeArmDropCommand extends Command {

    private final IntakeArmSystem system;
    private boolean isHolding;

    public IntakeArmDropCommand(IntakeArmSystem system) {
        this.system = system;
        addRequirements(system);
    }

    @Override
    public void initialize() {
        system.setTargetPosition(RobotMap.INTAKE_ARM_MIN_ANGLE_DEG);
        isHolding = true;
    }

    @Override
    public void execute() {
        if (isHolding && system.IsArmInPositionAndSteady(RobotMap.INTAKE_ARM_MIN_ANGLE_DEG)) {
            isHolding = false;
            system.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        system.stop();
    }

    @Override
    public boolean isFinished() {
        return system.IsArmInPositionAndSteady(RobotMap.INTAKE_ARM_MIN_ANGLE_DEG);
    }
}