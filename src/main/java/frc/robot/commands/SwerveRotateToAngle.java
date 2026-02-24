package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.Swerve;

public class SwerveRotateToAngle extends Command {

    private final Swerve swerve;
    private final double targetAngleDegrees;

    private final PIDController pidController;

    public SwerveRotateToAngle(Swerve swerve, double targetAngleDegrees) {
        this.swerve = swerve;
        this.targetAngleDegrees = targetAngleDegrees;

        pidController = new PIDController(RobotMap.SWERVE_PATH_ROTATE_PID.kP, 0,0);
        pidController.setTolerance(0.5, 0.1);
        pidController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        pidController.reset();
        pidController.setSetpoint(Math.toRadians(targetAngleDegrees));
    }

    @Override
    public void execute() {
        Rotation2d swerveRotation = swerve.getPose().getRotation();
        double power = pidController.calculate(swerveRotation.getRadians());
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, power, swerveRotation));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
