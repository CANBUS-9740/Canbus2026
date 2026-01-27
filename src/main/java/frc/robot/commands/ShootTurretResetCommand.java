package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShootTurretSystem;

public class ShootTurretResetCommand extends Command {
    private final ShootTurretSystem shootTurretSystem;

    public ShootTurretResetCommand(ShootTurretSystem shootTurretSystem) {
        this.shootTurretSystem = shootTurretSystem;

        addRequirements(shootTurretSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shootTurretSystem.setSpeed(RobotMap.SHOOT_TURRET_RESET_STATIC_SPEED);
    }

    @Override
    public boolean isFinished() {
        return shootTurretSystem.getLimitSwitchMin() || shootTurretSystem.getLimitSwitchMax() || shootTurretSystem.getLimitSwitchMiddle();
    }

    @Override
    public void end(boolean interrupted) {
        if (shootTurretSystem.getLimitSwitchMin()) {
            shootTurretSystem.setEncoderAngle(RobotMap.SHOOT_TURRET_MIN_ANGLE_DEGREES);
        } else if (shootTurretSystem.getLimitSwitchMax()) {
            shootTurretSystem.setEncoderAngle(RobotMap.SHOOT_TURRET_MAX_ANGLE_DEGREES);
        } else if (shootTurretSystem.getLimitSwitchMiddle()) {
            shootTurretSystem.setEncoderAngle(RobotMap.SHOOT_TURRET_MIDDLE_ANGLE_DEGREES);
        }
        shootTurretSystem.stop();
    }
}
