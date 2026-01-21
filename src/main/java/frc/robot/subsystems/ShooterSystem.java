package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final SparkMax shooterMotorMain ;
    private final RelativeEncoder speedEncoder;
    private final SparkMax shooterMotorFollow;
    private final Servo pitchMotor;

    public final SparkClosedLoopController sparkPIDcontroller;

    public ShooterSystem(){
        shooterMotorMain=new SparkMax(RobotMap.MAIN_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        shooterMotorFollow=new SparkMax(RobotMap.FOLLOWING_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        pitchMotor=new Servo(RobotMap.PITCH_MOTOR);

        sparkPIDcontroller = shooterMotorMain.getClosedLoopController();

        SparkMaxConfig config_lead= new SparkMaxConfig();
        SparkMaxConfig config_follow= new SparkMaxConfig();

        config_lead.closedLoop
                .p(RobotMap.KP)
                .i(RobotMap.KI)
                .d(RobotMap.KD);

        shooterMotorMain.configure(config_lead,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        config_follow.follow(shooterMotorMain,true);
        shooterMotorFollow.configure(config_follow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        speedEncoder=shooterMotorMain.getEncoder();
    }

    public void setWheelVoltage (int pow){
        shooterMotorMain.setVoltage(pow);
    }


}
