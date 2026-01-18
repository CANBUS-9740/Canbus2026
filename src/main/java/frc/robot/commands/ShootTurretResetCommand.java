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
        return shootTurretSystem.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        shootTurretSystem.setEncoderAngle(0);
        shootTurretSystem.stop();
    }
}
