package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeArmSystem;

public class IntakeArmDownSyndrome extends Command {

    private final IntakeArmSystem intakeArmSystem;

    public IntakeArmDownSyndrome(IntakeArmSystem intakeArmSystem) {
        this.intakeArmSystem = intakeArmSystem;
        addRequirements(intakeArmSystem);
    }

    @Override
    public void initialize() {
        intakeArmSystem.set(-0.2);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intakeArmSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return intakeArmSystem.getPositionDegrees() <= RobotMap.INTAKE_ARM_MIN_ANGLE_DEG;
    }
}
