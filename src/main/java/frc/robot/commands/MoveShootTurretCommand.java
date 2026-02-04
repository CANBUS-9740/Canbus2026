package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShootTurretSystem;

public class MoveShootTurretCommand extends Command {
    private final ShootTurretSystem shootTurretSystem;
    private final double targetAngle;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    public MoveShootTurretCommand(ShootTurretSystem shootTurretSystem, double targetAngle) {
        this.shootTurretSystem = shootTurretSystem;
        this.targetAngle = targetAngle;

        addRequirements(shootTurretSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Need to implement reference to swerve angle in the future

        if (targetAngle < RobotMap.SHOOT_TURRET_MAX_ANGLE_DEGREES && targetAngle > RobotMap.SHOOT_TURRET_MIN_ANGLE_DEGREES) {
            motionProfile = new TrapezoidProfile(RobotMap.SHOOT_TURRET_MOTION_PROFILE_CONSTRAINTS);
            motionProfileGoal = new TrapezoidProfile.State(targetAngle, 0);
            motionProfileSetPoint = new TrapezoidProfile.State(shootTurretSystem.getEncoderAngleInDegrees(), 0);
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);

            shootTurretSystem.setTargetPosition(motionProfileSetPoint.position);
        } else {
            shootTurretSystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return shootTurretSystem.isAtAngle(targetAngle);
    }

    @Override
    public void end(boolean interrupted) {
        shootTurretSystem.stop();
    }
}
