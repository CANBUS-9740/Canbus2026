package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class ShootCommand extends Command {
    private final ShooterSystem shooterSystem;
    private final double targetPitch;
    private final double targetRPM;
    private boolean setNewTarget;
    private boolean isAtPitch;

    private TrapezoidProfile pitchProfile;
    private TrapezoidProfile.State pitchProfileGoal;
    private TrapezoidProfile.State pitchProfileSetPoint;

    public ShootCommand(ShooterSystem shooterSystem, double targetPitch, double targetRPM) {
        this.shooterSystem = shooterSystem;
        this.targetPitch = targetPitch;
        this.targetRPM = targetRPM;

        addRequirements(shooterSystem);
    }

    @Override
    public void initialize() {
        setNewTarget = true;
        pitchProfile = new TrapezoidProfile(RobotMap.SHOOTER_PITCH_MOTION_PROFILE_CONSTRAINTS);
        pitchProfileGoal = new TrapezoidProfile.State(targetPitch, 0);
        pitchProfileSetPoint = new TrapezoidProfile.State(shooterSystem.getPitchAngleDegrees(), 0);
        shooterSystem.setShootVoltage(targetRPM / RobotMap.SHOOTER_MECHANISM_MAX_RPM);

    }

    @Override
    public void execute() {


        if (shooterSystem.isAtAngle(targetPitch)) {

            shooterSystem.setPitchPosition(targetPitch);
            isAtPitch = true;

        } else {
            pitchProfileSetPoint = pitchProfile.calculate(0.02, pitchProfileSetPoint, pitchProfileGoal);
            shooterSystem.setPitchPosition(pitchProfileSetPoint.position);
        }


        if (shooterSystem.getShooterVelocityRPM() >= targetRPM - 10 && isAtPitch) {
            shooterSystem.setFeederVoltage(RobotMap.SHOOTER_FEEDER_CONSTATNT);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean wasInterrupted) {
        shooterSystem.stopPitch();
        shooterSystem.stopShooterAndFeeder();
    }



}
