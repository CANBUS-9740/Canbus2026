package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterSystem extends SubsystemBase {
    private final SparkMax shooterMotorleft ;
    private final SparkMax shooterMotorright;
    private final Servo pitch;

    public ShooterSystem(){
        shooterMotorleft=new SparkMax(RobotMap.LEFT_SHOOTER_MOTOR, SparkLowLevel.MotorType.kBrushless);
        shooterMotorright=new SparkMax(RobotMap.RIGHT_SHOOTER_MOTOR,SparkLowLevel.MotorType.kBrushless);
        pitch=new Servo(RobotMap.PITCH_MOTOR);
        SparkMaxConfig config_lead= new SparkMaxConfig();
        SparkMaxConfig config_folllow= new SparkMaxConfig();
        config_folllow.follow(shooterMotorleft, true);
        shooterMotorleft.configure(config_lead, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotorright.configure(config_folllow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }
}
