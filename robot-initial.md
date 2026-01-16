The robot concept is made up of several systems, the following will describe the systems, capabilities and requirements for the robot.

## Systems

### Intake

The intake system is made up of two parts: the arm and the collector. It is used to collect balls from the floor into the storage tank.

#### Arm

The arm is used to lower and raise the collector. This is to keep the collector inside the robot frame as to keep it protected and to follow rule limitations.
This means that the arm has two position: raised and lowered. By default the arm must be raised, and while collecting the arm will be lowered for the duration of collection. 
The arm is made up of
- 1x _NEO v1.1_ motor to operate the arm
- 1x _SparkMax_ controlling the motor
- 1x _ThroughBore_ encoder in _Absolute_ mode to track the arm position

The arm must:
- Be able to keep the arm at raised position
- Be able to keep the arm at lower position
- Be able to track its own position
- Be able to lower or raise arm between the two positions as fast and as smoothly as possible

This implementation will be done with a _PID_ controller (running on the motor controller) to both keep the arm in place and to raise or lower it. The feedback sensor will be the absolute encoder and not the relative encoder.

Because the encoder is going to be connected to the robot via the _SparkMax_, accessing it is done as follows:
```java
private final SparkAbsoluteEncoder encoder;

public Subsystem() {
    ...
    encoder = motor.getAbsoluteEncoder();
}

public double getPositionDegrees() {
    return encoder.getPosition() * 360;
}
```

For pid control, use the `kPosition` control mode. Remember to configure the feedback sensor to the absolute encoder. See example [here](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#motor-position-closed-loop-control-spark-max) and [here](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#motor-position-closed-loop-control-spark-max).

#### Collector

The collector is used to push balls into the storage tanks on the robot. This is done by a line of wheels rotated by a motor fast enough to push 
the balls they come in contact with into the robot. The robot can then drive forward slowly to push balls in.
It is made up of
- 1x _NEO v1.1_ motor to rotate the wheels
- 1x _SparkMax_ controlling the motor

It must
- Be able to rotate inward to push balls into the storage

The implementation can use a contant speed which will be calibrated by manually testing.

### Storage

The storage is primarly used to store the balls collected until used for shooting. But it must have a way to move the balls around as to both feed them
into the shooter and to properly intake new balls without cloging. This requires some moving parts. More specfically, the floor of the tank is lined with rollers operated together
so as to move balls around. 

There are two sets of rollers: the general one which lines most of the tank and the shooter feed one, which lines up balls into the shooter.
The general is made up of
- 1x _NEO v1.1_ motor
- 1x _SparkMax_ controlling the motor

The feed roller is made up of 
- 1x _NEO v1.1_ motor
- 1x _SparkMax_ controlling the motor

The general rollers must be
- able to rotate in both direction in slow and high speeds.
- The specific speeds are constant but must be calibrated

The rollers will be used in several cases
- clear space for new intake
  - general rollers at slow speeds in the direction of the shooter
  - feed roller at slow speed in the opposite direction to prevent accidental feeding
- feed into the shooter
  - both general and feed rollers at high speed in the direction of the shooter
- unclog shooter feed
  - both rollers at high speed for 500ms in the opposite direction from the shooter to clear any obstacle in the shooter feed   

There are several sensors to provide indication on the contents of the storage
- 2x IR Limit Switch sensors placed in two specially selected positions to determine if there are any balls in the storage
- 1x Limelight Camera placed at the top part of the tank to see if it is overflowing. This will be done seperatly.

To access each IR sensor, define a `DigitalInput` device for each.
```java
private final DigitalInput irSensor;

public Subsystem() {
    ...
    irSensor = new DigitalInput(5);
}

public boolean doesSensorSeeSomething() {
    return irSensor.get();
}
```

### Turret

The turret provides an range of rotations for shooting at different directions. This provides flexiblity in shooting and increases the speed of firing. The turret 
itself is a large gear with a hole in the middle for balls to pass through into the shooter. The gear is in contact with another gear connected to a motor which will rotate it,

It is made up of
- 1x _NEO v1.1_ motor
- 1x _SparkMax_ controlling the motor
- 1x IR Limit Switch Sensor at position 0 for reset
- the exact gear ratio is still unknown

The turret must
- Be able to track its own position accurately
- Be able to hold a specific position
- Be able to quickly move between two positions

The motion can be implemented with _PID_ (integrated in motor). While the position track must have a reset at the start of a match to easily find the real 0 position due to the reliance on a relative sensor.

To access each IR sensor, define a `DigitalInput` device.
```java
private final DigitalInput irSensor;

public Subsystem() {
    ...
    irSensor = new DigitalInput(5);
}

public boolean doesSensorSeeSomething() {
    return irSensor.get();
}
```

For pid control, use the `kPosition` control mode. See example [here](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#motor-position-closed-loop-control-spark-max) and [here](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#motor-position-closed-loop-control-spark-max).

### Shooter

The shooter is resposibly for propelling the balls out of the robot and into the hub. It receives balls from the storage, and pushes them up through the turret and ejects them.
This is done in three parts: 1. propels the balls from the storage into the shooter mechnism itself using several rollers operating by a motor; 2. propels the ball out of the robot with a set of wheels. 3. A sliding edge to modify the firing angle of the ball.

The feed part is made up of:
- 1x _NEO v1.1_ motor
- 1x _SparkMax_ controlling the motor
- 1x IR Limit Switch Sensor at the entrance to detect balls feeding into the shooter

The shooter is made up of:
- 2x _NEO v1.1_ motor
- 2x _SparkMax_ controlling the motor
- The motors operate together to push the ball out as fast as possible

The slide is made up of:
- 1x _Servo_

The shooter process goes as such:
- Spin up shooter to shooting speed
- Adjust slide to shooting angle
- feed balls into shooter at a constant speed. There may be a need for a delay in feeding if the feeding is too fast.

So the shooter must:
- Be able to reach specific rotational speeds: use pid
- Be able to adjust firing angle: simple servo use
- feed balls: move at contant speed

The IR sensor is used to detect a feed-jam: i.e. no balls are being fed. In such a case we can unjam by operating the storage.

To access each IR sensor, define a `DigitalInput` device.
```java
private final DigitalInput irSensor;

public Subsystem() {
    ...
    irSensor = new DigitalInput(5);
}

public boolean doesSensorSeeSomething() {
    return irSensor.get();
}
```

To use servo, define a `Servo` device
```java
private final Servo servo;

public Subsystem() {
    ...
    servo = new Servo(1);
}

public double getServoPosDegrees() {
    return servo.getAngle();
}

public void setServoPosDegrees(double degrees) {
    return servo.setAngle(degrees);
}
```

For pid control, use the `kVelocity` control mode. See example [here](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#motor-position-closed-loop-control-spark-max) and [here](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#motor-position-closed-loop-control-spark-max).

### Climb

The climb system is used to reach the first climbing height by relying on two telescopic hooks. Each of those hooks can be extended or retracted to reach the poll and then pull the robot up. It is made up of
- 2x _NEO v1.1_ motors, one for each arm
- 2x _SparkMax_ motor controllers, one for each arm
- 2x Magnetic Limit Switch, one for each arm

To calculated the length of the arm, it will use the integrated encoders in the motors to count rotations. Each rotation can be translated to a set amount of length for the arms. Since the encoder is relative, 

the exact length at start time is unknown, so use the two limit switches to reset the arms to the 0 position.

It must be able to
- reset the arms to zero position without relying on the encoder
- extending the arms to specific length based on the encoder reading with _PID_.

For pid control, use the `kPosition` control mode. See example [here](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#motor-position-closed-loop-control-spark-max) and [here](https://github.com/tomtzook/frc-learn-docs/blob/master/robot-code-snippets.md#motor-position-closed-loop-control-spark-max).

To access each Limit Switch sensor, define a `DigitalInput` device.
```java
private final DigitalInput switchSensor;

public Subsystem() {
    ...
    switchSensor = new DigitalInput(5);
}

public boolean isSwitchPressed() {
    return switchSensor.get();
}
```

## Automations

The following automation capabilities are wanted
- Go to position in field: use pathplanner to go to any wanted position on the field
  - this will require localization of the robot position with odometry and april tag tracking 
- Automatic shooting calculation: calculate firing speed and angle using physics based on distance to target
  - this will require implementation of the algorithm + sensor information required by the calculation 
- Shooter feed jam detection and handling: detect when feed into shooter is jammed and unjam it
- Automatic balls on floor detection: detect ball positions on the floor with limelight
- Storage empty/full detection: detect when storage is too full or empty of balls
- Automatic climbing: automatically climb 
- Constant tracking of hub with turret: keep turret aligned on hub for immediate shooting
- Proper ball feeding into shooter: make sure to feed with a slight delay to keep firing consistent
- Automatic intake: be able to collect balls from the floor without human interference.
