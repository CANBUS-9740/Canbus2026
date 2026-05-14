package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        //intakeArmSystem.setTargetPosition(RobotMap.INTAKE_ARM_MIN_ANGLE_DEG);
        intakeArmSystem.set(-0.2);
    }

    @Override
    public void execute() {
        if (intakeArmSystem.getPositionDegrees() <= 10) {
            intakeArmSystem.stop();
            intakeArmSystem.set(0.005);
        }
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
