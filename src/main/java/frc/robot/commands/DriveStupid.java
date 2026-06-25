package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class DriveStupid extends Command {

    private final Swerve swerve;
    private double lastPos;
    private double velocity;
    private double distance;

    public  DriveStupid(Swerve swerve, double velocity, double distance) {
        this.velocity = velocity;
        this.distance = distance;

        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        //lastPos = swerve.swerveDrive.getModules()[0].getPosition().distanceMeters;
        lastPos = swerve.getPose().getX();
    }

    @Override
    public void execute() {
        double currentDis = Math.abs(swerve.swerveDrive.getPose().getX() - lastPos);
        swerve.swerveDrive.drive(new ChassisSpeeds(velocity, 0, 0));
    }

    @Override
    public boolean isFinished() {
        //return Math.abs(swerve.swerveDrive.getModules()[0].getPosition().distanceMeters - lastPos) >= distan
        return Math.abs(swerve.swerveDrive.getPose().getX() - lastPos) >= distance;

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Bitch");
        swerve.stop();
    }
}