package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.DynamicShooterSystem;

public class ShootCommand extends Command {
    private final DynamicShooterSystem dynamicShooterSystem;
    private final double targetPitch;
    private final double targetRPM;
    private boolean setNewTarget;
    private boolean isAtPitch;

    private TrapezoidProfile pitchProfile;
    private TrapezoidProfile.State pitchProfileGoal;
    private TrapezoidProfile.State pitchProfileSetPoint;

    public ShootCommand(DynamicShooterSystem dynamicShooterSystem, double targetPitch, double targetRPM) {
        this.dynamicShooterSystem = dynamicShooterSystem;
        this.targetPitch = targetPitch;
        this.targetRPM = targetRPM;

        addRequirements(dynamicShooterSystem);
    }

    @Override
    public void initialize() {
        setNewTarget = true;
        pitchProfile = new TrapezoidProfile(RobotMap.SHOOTER_PITCH_MOTION_PROFILE_CONSTRAINTS);
        pitchProfileGoal = new TrapezoidProfile.State(targetPitch, 0);
        pitchProfileSetPoint = new TrapezoidProfile.State(dynamicShooterSystem.getPitchAngleDegrees(), 0);
        dynamicShooterSystem.setShootVoltage(targetRPM / RobotMap.SHOOTER_MECHANISM_MAX_RPM);

    }

    @Override
    public void execute() {


        if (dynamicShooterSystem.isAtAngle(targetPitch)) {

            dynamicShooterSystem.setPitchPosition(targetPitch);
            isAtPitch = true;

        } else {
            pitchProfileSetPoint = pitchProfile.calculate(0.02, pitchProfileSetPoint, pitchProfileGoal);
            dynamicShooterSystem.setPitchPosition(pitchProfileSetPoint.position);
        }


        if (dynamicShooterSystem.getShooterVelocityRPM() >= targetRPM - 10 && isAtPitch) {
            dynamicShooterSystem.setFeederVoltage(RobotMap.SHOOTER_FEEDER_CONSTATNT);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean wasInterrupted) {
        dynamicShooterSystem.stopPitch();
        dynamicShooterSystem.stopShooterAndFeeder();
    }



}
