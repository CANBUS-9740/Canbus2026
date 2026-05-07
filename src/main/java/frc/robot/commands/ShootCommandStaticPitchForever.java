package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StaticShooterSystem;

public class ShootCommandStaticPitchForever extends Command {
    private final StaticShooterSystem staticShooterSystem;
    private final double targetRPM;

    public ShootCommandStaticPitchForever(StaticShooterSystem staticShooterSystem, double targetRPM) {
        this.staticShooterSystem = staticShooterSystem;
        this.targetRPM = targetRPM;
        addRequirements(staticShooterSystem);
    }

    @Override
    public void initialize() {
        staticShooterSystem.setShootSpeed(targetRPM);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}