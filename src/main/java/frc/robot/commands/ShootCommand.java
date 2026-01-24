package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ShooterSystem;

public class ShootCommand extends Command {
    private final ShooterSystem shooterSystem;
    private double targetPitch;
    private double RPM,deltaRPM;
    private boolean setnewtarget,isAtPitch;


    public ShootCommand(ShooterSystem shooterSystem, double targetPitch, double RPM) {
        this.shooterSystem=shooterSystem;
        this.targetPitch=targetPitch;
        this.RPM=RPM;

        addRequirements(shooterSystem);
    }
    @Override
    public void initialize(){
        setnewtarget=true;


    }
    @Override
    public void execute(){
        if(setnewtarget){
            setnewtarget=false;

            if(getIsNear()){
                shooterSystem.stop_Angle();
            }
            else{

            }
        }

        shooterSystem.setWheelVoltage(RPM/ RobotMap.MAX_RPM);
        if(shooterSystem.getRPM()>=RPM-10){
            shooterSystem.setFeederVoltage(RobotMap.FEEDER_CONSTATNT);
        }
    }

    @Override
    public boolean isFinished(){return false;}

    @Override
    public void end(boolean wasInterrupted){}

    public boolean getIsNear(){
        if(setnewtarget){
            return false;
        }
        else{
            return shooterSystem.isAtAngle(targetPitch);
        }
    }
}
