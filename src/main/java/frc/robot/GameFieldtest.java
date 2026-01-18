package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class GameFieldtest {

    enum Hub{
        frontside_left(9,25),
        frontside_right(10,26),
        rightside_left(11,27),
        rightside_right(2,18),
        backside_left(3,19),
        backside_right(4,20),
        leftside_left(5,21),
        leftside_right(8,24);

        public final int ApriltagIdred;
        public final int ApriltagIdblue;

        Hub(int ApriltagIdred, int ApriltagIdblue) {
            this.ApriltagIdred = ApriltagIdred;
            this.ApriltagIdblue = ApriltagIdblue;
        }
    }

    enum Trench{
        leftside_front(7,23),
        leftside_back(6,22),
        rightside_front(12,28),
        rightside_back(1,17);

        public final int ApriltagIdred;
        public final int ApriltagIdblue;

        Trench(int ApriltagIdred, int ApriltagIdblue) {
            this.ApriltagIdred = ApriltagIdred;
            this.ApriltagIdblue = ApriltagIdblue;
        }
    }

    enum Depot{
        leftside(13,29),
        rightside(14,30);

        public final int ApriltagIdred;
        public final int ApriltagIdblue;

        Depot(int ApriltagIdred, int ApriltagIdblue) {
            this.ApriltagIdred = ApriltagIdred;
            this.ApriltagIdblue = ApriltagIdblue;
        }
    }

    enum Tower{
        leftside(15,31),
        rightside(16,32);

        public final int ApriltagIdred;
        public final int ApriltagIdblue;

        Tower(int ApriltagIdred, int ApriltagIdblue) {
            this.ApriltagIdred = ApriltagIdred;
            this.ApriltagIdblue = ApriltagIdblue;
        }
    }

}
