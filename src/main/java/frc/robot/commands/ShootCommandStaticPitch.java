package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.StaticShooterSystem;
import frc.robot.subsystems.StorageSystem;

public class ShootCommandStaticPitch extends Command {
    private final StorageSystem storageSystem;
    private final StaticShooterSystem staticShooterSystem;
    //private final double targetDistance

    private double targetRPM;
    private double shooterDistance;
    private double shooterDistanceOffset;

    private Timer delayTime;
    private boolean isCompensating;
    private boolean justStarted;

    public ShootCommandStaticPitch(StorageSystem storageSystem, StaticShooterSystem staticShooterSystem, double shooterDistance) {
        this.storageSystem = storageSystem;
        this.staticShooterSystem = staticShooterSystem;
        this.shooterDistance = (shooterDistance - 0.3) * (0.2 * Math.pow(shooterDistance, 2) - 1.83 * shooterDistance + 6.675);
        this.targetRPM = staticShooterSystem.calculateFiringSpeedRpm(this.shooterDistance, 70) ;

        this.justStarted = true;
        this.isCompensating = false;
        this.delayTime = new Timer();

        addRequirements(staticShooterSystem, storageSystem);
    }

    @Override
    public void initialize() {
        this.justStarted = true;
        this.isCompensating = false;
        this.delayTime = new Timer();

        SmartDashboard.putBoolean("Running1", false);
        SmartDashboard.putBoolean("Running2", false);
        // shooterDistanceOffset = 0.2 * Math.pow(shooterDistance, 2) -1.83 * shooterDistance + 6.675;
        // shooterDistance = (shooterDistance - 0.3) * shooterDistanceOffset;
        // targetRPM = staticShooterSystem.calculateFiringSpeedRpm(shooterDistance ,70);
        staticShooterSystem.setShootSpeed(targetRPM);

        SmartDashboard.putNumber("ShooterTarget", targetRPM);
        SmartDashboard.putNumber("ShooterDistanceTarget", shooterDistance);

        delayTime.start();
    }

    @Override
    public void execute() {
        if (MathUtil.isNear(targetRPM, staticShooterSystem.getShooterVelocityRPM(), 5) && delayTime.get() > 1.5) {
            SmartDashboard.putBoolean("Running1", true);
            justStarted = false;
            staticShooterSystem.startFeederMotor();
            storageSystem.moveGeneralRollers(RobotMap.STORAGE_GENERAL_ROLLERS_FORWARD_HIGH_SPEED);

            if (isCompensating) {
                staticShooterSystem.setShootSpeed(targetRPM);
                isCompensating = false;
            }
        } else if (!isCompensating && !justStarted) {
            SmartDashboard.putBoolean("Running2", true);
            staticShooterSystem.shootSpeedCompensation(targetRPM);
            isCompensating = true;
        }

        SmartDashboard.putBoolean("if", MathUtil.isNear(targetRPM, staticShooterSystem.getShooterVelocityRPM(), 5));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end (boolean interrupted) {
        staticShooterSystem.stopShooterAndFeeder();
        storageSystem.stopMotors();
        SmartDashboard.putBoolean("Running1", false);
        SmartDashboard.putBoolean("Running2", false);
    }
}