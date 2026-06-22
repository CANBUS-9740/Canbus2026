package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class DriveStupid extends Command {

    private final Swerve swerve;
    private double lastPos;

    public DriveStupid(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        lastPos = swerve.swerveDrive.getModules()[0].getPosition().distanceMeters;
    }

    @Override
    public void execute() {
        swerve.swerveDrive.drive(new ChassisSpeeds(0.5, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.swerveDrive.getModules()[0].getPosition().distanceMeters - lastPos) >= 1.5;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Bitch");
        swerve.swerveDrive.drive(new ChassisSpeeds(0, 0, 0));
    }
}