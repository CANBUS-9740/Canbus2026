package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import org.dyn4j.geometry.Vector3;

public class SimulatedBall {

    private static final Vector3 HUB_CENTER = new Vector3(4.69, 4.0, 2.642);
    private static final double HUB_RADIUS = 1.372 / 2;
    private static final double HUB_AABB_HEIGHT = 0.2;

    private static final Vector3 AABB_HUB_BOX_MIN = new Vector3(
            HUB_CENTER.x - HUB_RADIUS, HUB_CENTER.y - HUB_RADIUS, HUB_CENTER.z - HUB_AABB_HEIGHT);
    private static final Vector3 AABB_HUB_BOX_MAX = new Vector3(
            HUB_CENTER.x + HUB_RADIUS, HUB_CENTER.y + HUB_RADIUS, HUB_CENTER.z);

    private final FieldObject2d fieldObject;

    private final NetworkTableEntry positionEntry;
    private final double[] positionEntryInfo;
    private final NetworkTableEntry velocityEntry;
    private final double[] velocityEntryInfo;
    private final NetworkTableEntry accelerationEntry;
    private final double[] accelerationEntryInfo;
    private final NetworkTableEntry enteredHubEntry;

    private Vector3 position;
    private Vector3 velocity;
    private Vector3 acceleration;
    private boolean touchedFloor;
    private boolean enteredHub;

    public SimulatedBall(FieldObject2d fieldObject, NetworkTable table) {
        this.fieldObject = fieldObject;

        positionEntry = table.getEntry("position");
        positionEntryInfo = new double[3];
        positionEntry.setDoubleArray(positionEntryInfo);

        velocityEntry = table.getEntry("velocity");
        velocityEntryInfo = new double[3];
        velocityEntry.setDoubleArray(velocityEntryInfo);

        accelerationEntry = table.getEntry("acceleration");
        accelerationEntryInfo = new double[3];
        accelerationEntry.setDoubleArray(accelerationEntryInfo);

        enteredHubEntry = table.getEntry("InHub");
        enteredHubEntry.setBoolean(false);

        position = new Vector3();
        velocity = new Vector3();
        acceleration = new Vector3();
        touchedFloor = false;
        enteredHub = false;

        fieldObject.setPose(new Pose2d(position.x, position.y, Rotation2d.kZero));
    }

    public Vector3 getPosition() {
        return position;
    }

    public void setPosition(Vector3 position) {
        this.position = position;
        updateEntry(positionEntry, positionEntryInfo, position);
    }

    public void setVelocity(Vector3 velocity) {
        this.velocity = velocity;
        updateEntry(velocityEntry, velocityEntryInfo, velocity);
    }

    public void setAcceleration(Vector3 acceleration) {
        this.acceleration = acceleration;
        updateEntry(accelerationEntry, accelerationEntryInfo, acceleration);
    }

    public void update(double dt) {
        // x = x0 + v0 t + 0.5 a t^2
        // v = v0 + a t
        Vector3 newVelocity = velocity.add(acceleration.product(dt));
        Vector3 newPosition = position.add(velocity.product(dt)).add(acceleration.product(dt * dt).product(0.5));

        if (!enteredHub && checkEnteringHub(position, newPosition, AABB_HUB_BOX_MIN, AABB_HUB_BOX_MAX)) {
            enteredHub = true;
            enteredHubEntry.setBoolean(true);
        }

        if (!touchedFloor && newPosition.z <= 0) {
            newPosition.z = 0;
            newVelocity = new Vector3();
            acceleration = new Vector3();
            touchedFloor = true;
        }

        updateEntry(positionEntry, positionEntryInfo, newPosition);
        updateEntry(velocityEntry, velocityEntryInfo, newVelocity);
        fieldObject.setPose(new Pose2d(newPosition.x, newPosition.y, Rotation2d.kZero));

        position = newPosition;
        velocity = newVelocity;
    }

    private void updateEntry(NetworkTableEntry entry, double[] info, Vector3 data) {
        info[0] = data.x;
        info[1] = data.y;
        info[2] = data.z;
        entry.setDoubleArray(info);
    }

    private boolean checkEnteringHub(Vector3 startPos, Vector3 endPose, Vector3 minBox, Vector3 maxBox) {
        if (!checkAxisCollision(startPos.x, endPose.x, minBox.x, maxBox.x)) {
            return false;
        }
        if (!checkAxisCollision(startPos.y, endPose.y, minBox.y, maxBox.y)) {
            return false;
        }
        if (!checkAxisCollision(startPos.z, endPose.z, minBox.z, maxBox.z)) {
            return false;
        }

        return true;
    }

    private boolean checkAxisCollision(double start, double end, double minBox, double maxBox) {
        double tMin = 0.0;
        double tMax = 1.0;

        double invDir = 1.0 / (end - start);

        double t0 = (minBox - start) * invDir;
        double t1 = (maxBox - start) * invDir;

        // Swap if the ray is moving in the negative direction
        if (invDir < 0.0) {
            double temp = t0;
            t0 = t1;
            t1 = temp;
        }

        tMin = Math.max(tMin, t0);
        tMax = Math.min(tMax, t1);

        // If the entry time is after the exit time, there is no intersection
        if (tMax < tMin) {
            return false;
        }

        return true;
    }
}
