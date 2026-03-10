package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.StaticShooterSystem;

public class ShootCommandStaticPitch extends Command {
    private final StaticShooterSystem staticShooterSystem;
    private final double targetRPM;

    public ShootCommandStaticPitch(StaticShooterSystem staticShooterSystem, double targetRPM) {
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
        staticShooterSystem.stopShooterAndFeeder();
    }
}
