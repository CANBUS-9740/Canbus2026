package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
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

    private boolean isCompensating=false;
    private boolean justStarted=true;
    private double compensationDifference;
    private Timer delaytime;

    public ShootStrafeTest(StorageSystem storageSystem, StaticShooterSystem staticShooterSystem, double shooterDistance) {
        this.storageSystem = storageSystem;
        this.staticShooterSystem = staticShooterSystem;
        this.shooterDistance = shooterDistance;
        addRequirements(staticShooterSystem, storageSystem);
        delaytime = new Timer();
    }

    @Override
    public void initialize() {
        shooterDistanceOffset = 0.2*Math.pow(shooterDistance,2)-1.83*shooterDistance+6.675;
        shooterDistance = shooterDistance*shooterDistanceOffset;
        targetRPM = staticShooterSystem.calculateFiringSpeedRpm(shooterDistance ,70);
        staticShooterSystem.setShootSpeed(targetRPM);
        SmartDashboard.putNumber("ShooterTarget", targetRPM);
        delaytime.start();
    }

    @Override
    public void execute() {
        if (MathUtil.isNear(targetRPM ,staticShooterSystem.getShooterVelocityRPM(), 5)&&delaytime.get()>1.5){
            justStarted=false;
            staticShooterSystem.startFeederMotor();
            storageSystem.moveGeneralRollers(RobotMap.STORAGE_GENERAL_ROLLERS_FORWARD_HIGH_SPEED);
            if(isCompensating){
                staticShooterSystem.setShootSpeed(targetRPM);
                isCompensating=false;
            }
        }
        else{
            if(!isCompensating && !justStarted){
                staticShooterSystem.shootSpeedCompensation(targetRPM);
                isCompensating=true;
            }
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