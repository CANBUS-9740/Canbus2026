package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeCollectorSystem;

public class IntakeCollectorUnjamCommand extends Command {
    private final IntakeCollectorSystem intakeCollectorSystem;

    public IntakeCollectorUnjamCommand(IntakeCollectorSystem intakeCollectorSystem) {
        this.intakeCollectorSystem = intakeCollectorSystem;
        addRequirements(intakeCollectorSystem);
    }

    @Override
    public void initialize() {
        intakeCollectorSystem.move(-RobotMap.COLLECTOR_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        intakeCollectorSystem.stop();
    }
}
