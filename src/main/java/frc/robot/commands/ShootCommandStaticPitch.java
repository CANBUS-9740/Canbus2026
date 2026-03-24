package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.DynamicShooterSystem;
import frc.robot.subsystems.StaticShooterSystem;
import frc.robot.subsystems.StorageSystem;

public class ShootCommandStaticPitch extends Command {
    private final StaticShooterSystem staticShooterSystem;
    private final StorageSystem storageSystem;
    private final double targetRPM;

    public ShootCommandStaticPitch(StaticShooterSystem staticShooterSystem,StorageSystem storageSystem, double targetRPM) {
        this.staticShooterSystem = staticShooterSystem;
        this.storageSystem = storageSystem;
        this.targetRPM = targetRPM;
         addRequirements(staticShooterSystem);
    }

    @Override
    public void initialize() {

        staticShooterSystem.setShootSpeed(targetRPM);
    }
    @Override
    public void execute() {
        if(staticShooterSystem.getShooterVelocityRPM() >= targetRPM-10) {
            staticShooterSystem.setFeederVoltage(RobotMap.SHOOTER_FEEDER_CONSTATNT);

        }
    }
    @Override
    public boolean isFinished() {
        return storageSystem.atLeast1Ball();
    }
    @Override
    public void end(boolean interrupted) {
        staticShooterSystem.stopShooterAndFeeder();
    }
}
