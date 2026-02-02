package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.sim.ShootTurretSim;

public class ShootTurretSystem extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkLimitSwitch forwardLimitSwitch;
    private final SparkLimitSwitch reverseLimitSwitch;
    private final DigitalInput limitSwitchCenter;

    private final ShootTurretSim shootTurretSim;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d ligament;

    public ShootTurretSystem() {
        motor = new SparkMax(RobotMap.SHOOT_TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getEncoder();
        forwardLimitSwitch = motor.getForwardLimitSwitch();
        reverseLimitSwitch = motor.getReverseLimitSwitch();

        limitSwitchCenter = new DigitalInput(RobotMap.SHOOT_TURRET_LIMIT_SWITCH_CENTER_PORT);

        SparkMaxConfig config = new SparkMaxConfig();
        config.limitSwitch
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .forwardLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchTriggerBehavior(LimitSwitchConfig.Behavior.kStopMovingMotor);
        config.closedLoop
                .p(RobotMap.SHOOT_TURRET_PID_P)
                .i(RobotMap.SHOOT_TURRET_PID_I)
                .d(RobotMap.SHOOT_TURRET_PID_D)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.encoder
                .velocityConversionFactor(1 / RobotMap.SHOOT_TURRET_GEAR_RATIO)
                .positionConversionFactor(1 / RobotMap.SHOOT_TURRET_GEAR_RATIO);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (Robot.isSimulation()) {
            shootTurretSim = new ShootTurretSim(motor, limitSwitchCenter);
        } else {
            shootTurretSim = null;
        }

        mechanism = new Mechanism2d(5, 5);
        MechanismRoot2d root = mechanism.getRoot("TurretCenter", 2.5, 2.5);
        ligament = root.append(new MechanismLigament2d("TurretDir", 2, 0, 5, new Color8Bit(Color.kRed)));
        SmartDashboard.putData("Turret", mechanism);
    }

    public void setTargetPosition(double positionDegrees) {
        motor.getClosedLoopController().setSetpoint(positionDegrees / 360.0, SparkBase.ControlType.kPosition);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean getLimitSwitchMin() {
        return forwardLimitSwitch.isPressed();
    }

    public boolean getLimitSwitchMax() {
        return reverseLimitSwitch.isPressed();
    }

    public boolean getLimitSwitchCenter() {
        return limitSwitchCenter.get();
    }

    public void setEncoderAngle(double angle) {
        encoder.setPosition(angle / 360);
    }

    public double getEncoderAngleInDegrees() {
        double newValue = (encoder.getPosition() * 360) % 360;
        setEncoderAngle(newValue);
        return newValue;
    }

    public boolean isAtAngle(double angle) {
        return MathUtil.isNear(angle, getEncoderAngleInDegrees(), RobotMap.SHOOT_TURRET_IS_NEAR_TOLERANCE_DEGREES) & encoder.getVelocity() < RobotMap.SHOOT_TURRET_IS_NEAR_VELOCITY_RPM;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("ShootTurretLimitSwitchMin", getLimitSwitchMin());
        SmartDashboard.putBoolean("ShootTurretLimitSwitchMax", getLimitSwitchMax());
        SmartDashboard.putBoolean("ShootTurretLimitSwitchMiddle", getLimitSwitchCenter());
        SmartDashboard.putNumber("ShootTurretEncoderInDegrees", getEncoderAngleInDegrees());
        ligament.setAngle(getEncoderAngleInDegrees() + 90);
    }

    @Override
    public void simulationPeriodic() {
        shootTurretSim.update();
    }
}
