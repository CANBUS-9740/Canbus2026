package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.StorageSystem;

public class StorageCommand extends Command {
    StorageSystem storageSystem;

    public StorageCommand(StorageSystem storageSystem) {
        this.storageSystem = storageSystem;
        addRequirements();
    }

    public void initialize() {

    }


    public void execute() {
        storageSystem.moveMotor1(RobotMap.StorageMotor1Speed); //feed and storage
        storageSystem.moveMotor2(RobotMap.StorageMotor2Speed);
    }


    public void end(boolean interrupted) {
        storageSystem.stopMotors();
    }


    public boolean isFinished() {
        return false;
    }
}
