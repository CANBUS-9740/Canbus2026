package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShootTurretSystem;

public class MoveShootTurretCommand extends Command {
    private final ShootTurretSystem shootTurretSystem;
    private double targetAngle;
    private boolean targetChanged;

    public MoveShootTurretCommand(ShootTurretSystem shootTurretSystem) {
        this.shootTurretSystem = shootTurretSystem;

        addRequirements(shootTurretSystem);
    }

    @Override
    public void initialize() {
        targetChanged = false;
        targetAngle = 0;
    }

    @Override
    public void execute() {
        // Need to implement reference to swerve angle in the future

        if (targetChanged) {
            shootTurretSystem.setPosition(targetAngle);
            targetChanged = false;
        }

        if (shootTurretSystem.getLimitSwitch()) {
            shootTurretSystem.setEncoderAngle(0);
        }

        SmartDashboard.putBoolean("ShootTurretIsAtAngle", getIsNearTarget());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shootTurretSystem.stop();
    }

    public void setAngleInDegrees(double angle) {
        if (angle < RobotMap.SHOOT_TURRET_MAX_ANGLE || angle > RobotMap.SHOOT_TURRET_MIN_ANGLE) {
            this.targetAngle = angle;
            targetChanged = true;
        } else {
            DriverStation.reportWarning("Request to set angle out of range", true);
        }
    }

    public boolean getIsNearTarget() {
        return shootTurretSystem.isAtAngle(targetAngle);
    }
}
