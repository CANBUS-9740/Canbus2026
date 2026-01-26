package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.StorageSystem;

public class StorageBothRollersBackwardsCommand extends Command {

    private final StorageSystem storageSystem;

    public StorageBothRollersBackwardsCommand(StorageSystem storageSystem) {
        this.storageSystem = storageSystem;
        addRequirements(storageSystem);
    }

    public void initialize() {
        storageSystem.moveGeneralRollers(RobotMap.STORAGE_GENERAL_ROLLERS_BACKWARDS_LOW_SPEED);
        storageSystem.moveFeedRollers(RobotMap.STORAGE_FEED_ROLLERS_BACKWARDS_LOW_SPEED);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        storageSystem.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


