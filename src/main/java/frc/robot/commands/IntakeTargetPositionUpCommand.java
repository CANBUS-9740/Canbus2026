package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeArmSystem;

public class IntakeTargetPositionUpCommand extends Command {
    private final IntakeArmSystem intakeArmSystem;

    public IntakeTargetPositionUpCommand(IntakeArmSystem intakeArmSystem) {
        this.intakeArmSystem = intakeArmSystem;

        addRequirements(intakeArmSystem);
    }

    @Override
    public void initialize() {
        intakeArmSystem.setTargetPosition(RobotMap.INTAKE_ARM_MAX_ANGLE_DEG);
    }
}