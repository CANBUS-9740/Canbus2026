package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class ShootCommand extends Command {
    private final ShooterSystem shooterSystem;
    private double pitch,deltaPitch;
    private double RPM,deltaRPM;
    public ShootCommand(ShooterSystem shooterSystem, double pitch, double RPM) {
        this.shooterSystem=shooterSystem;
        this.pitch=pitch;
        this.RPM=RPM;
    }
    @Override
    public void initialize(){
        deltaPitch=shooterSystem.getRotations()-pitch;


    }
    @Override
    public void execute(){
        shooterSystem.setWheelVoltage(RPM/ RobotMap.MAX_RPM);
        if(shooterSystem.getRPM()>=RPM-10){
            shooterSystem.setFeederVoltage(RobotMap.FEEDER_CONSTATNT);
        }
    }

    @Override
    public boolean isFinished(){return false;}

    @Override
    public void end(boolean wasInterrupted){}

}
