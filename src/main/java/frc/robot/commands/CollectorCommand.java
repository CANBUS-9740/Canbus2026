package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.CollectorSystem;

public class CollectorCommand extends Command {
    CollectorSystem intakeCollectorSystem;
    public CollectorCommand(CollectorSystem intakeCollectorSystem) {
        this.intakeCollectorSystem = intakeCollectorSystem;
        addRequirements();
    }

    public void initialize() {

    }


    public void execute() {
        intakeCollectorSystem.move(RobotMap.CollectorSpeed);
    }


    public void end(boolean interrupted) {
        intakeCollectorSystem.stop();
    }


    public boolean isFinished() {
        return false;
    }


}
