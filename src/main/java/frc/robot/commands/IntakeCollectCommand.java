package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.IntakeCollectorSystem;

public class IntakeCollectCommand extends Command {
    private final IntakeCollectorSystem intakeCollectorSystem;
    public IntakeCollectCommand(IntakeCollectorSystem intakeCollectorSystem) {
        this.intakeCollectorSystem = intakeCollectorSystem;
        addRequirements(intakeCollectorSystem);
    }

    @Override
    public void initialize() {
        intakeCollectorSystem.move(RobotMap.COLLECTOR_SPEED);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intakeCollectorSystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
