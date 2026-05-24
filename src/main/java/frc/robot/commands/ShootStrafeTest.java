package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.StaticShooterSystem;
import frc.robot.subsystems.StorageSystem;

public class ShootStrafeTest extends Command {
    private final StorageSystem storageSystem;
    private final StaticShooterSystem staticShooterSystem;
    //private final double targetDistance

    private double targetRPM;
    private double shooterDistance;
    private double shooterDistanceOffset;

    public ShootStrafeTest(StorageSystem storageSystem, StaticShooterSystem staticShooterSystem, double shooterDistance) {
        this.storageSystem = storageSystem;
        this.staticShooterSystem = staticShooterSystem;
        this.shooterDistance = shooterDistance;
        addRequirements(staticShooterSystem, storageSystem);
    }

    @Override
    public void initialize() {
        shooterDistanceOffset = 0.2*Math.pow(shooterDistance,2)-1.83*shooterDistance+6.675;
        shooterDistance = shooterDistance*shooterDistanceOffset;
        targetRPM = staticShooterSystem.calculateFiringSpeedRpm(shooterDistance ,70);
        staticShooterSystem.setShootSpeed(targetRPM);
        SmartDashboard.putNumber("ShooterTarget", targetRPM);
    }

    @Override
    public void execute() {
        if (MathUtil.isNear(targetRPM ,staticShooterSystem.getShooterVelocityRPM(), 5)){
            staticShooterSystem.startFeederMotor();
            storageSystem.moveGeneralRollers(RobotMap.STORAGE_GENERAL_ROLLERS_FORWARD_HIGH_SPEED);
        }
        else{
            staticShooterSystem.startFeederMotor();
            storageSystem.moveGeneralRollers(0);
        }

        SmartDashboard.putBoolean("if", MathUtil.isNear(targetRPM ,staticShooterSystem.getShooterVelocityRPM(), 5));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        staticShooterSystem.stopShooterAndFeeder();
        storageSystem.stopMotors();
    }
}