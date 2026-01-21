package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSystem;

public class IntakeArmPositionCommand extends Command {
    private final IntakeArmSystem intakeArmSystem;
    private final double targetPositionDegrees;

    public IntakeArmPositionCommand(IntakeArmSystem intakeArmSystem, double targetPositionDegrees) {
        this.intakeArmSystem= intakeArmSystem;
        this.targetPositionDegrees = targetPositionDegrees;
        addRequirements(intakeArmSystem);
    }
    @Override
    public void initialize() {
        intakeArmSystem.setTargetPosition(targetPositionDegrees);
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
        return false;
    }



}


