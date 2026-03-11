package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.StorageSystem;

public class StorageFeedToShooterCommand extends Command {
    private final StorageSystem storageSystem;

    public StorageFeedToShooterCommand(StorageSystem storageSystem) {
        this.storageSystem = storageSystem;
        addRequirements(storageSystem);
    }
    @Override
    public void initialize() {
        storageSystem.moveGeneralRollers(RobotMap.STORAGE_GENERAL_ROLLERS_FORWARD_HIGH_SPEED); //feed and storage
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
