package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

        pidController = new PIDController(RobotMap.SWERVE_PATH_ROTATE_PID.kP, RobotMap.SWERVE_PATH_ROTATE_PID.kI, RobotMap.SWERVE_PATH_ROTATE_PID.kD);
        pidController.setTolerance(Math.toRadians(1), 0.1);
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

        SmartDashboard.putNumber("SwerveRotateCurrent", swerveRotation.getDegrees());
        SmartDashboard.putNumber("SwerveRotateTarget", targetAngleDegrees);

        double power = pidController.calculate(swerveRotation.getRadians());
        if (!MathUtil.isNear(0, power, 0.001) && Math.abs(power) < 0.2) {
            power = Math.signum(power) * 0.2;
        }
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