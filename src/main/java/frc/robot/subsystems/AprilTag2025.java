package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;

import java.util.List;

public interface AprilTag2025 {
    /**
     * LINK OF ALL THE APRIL TAG POSES FROM JSON FILE
     * <a href="https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape-welded.json">...</a>
     */
    List<AprilTag> LIST = List.of(
            new AprilTag(1, new Pose3d(16.697198, 0.65532, 1.4859, new Rotation3d(new Quaternion(0.4539904997395468, 0.0, 0.0, 0.8910065241883678)))),
            new AprilTag(2, new Pose3d(16.697198, 7.39648, 1.4859, new Rotation3d(new Quaternion(-0.45399049973954675, -0.0, 0.0, 0.8910065241883679)))),
            new AprilTag(3, new Pose3d(11.56081, 8.05561, 1.30175, new Rotation3d(new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
            new AprilTag(4, new Pose3d(9.27608, 6.137656, 1.867916, new Rotation3d(new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074, 0.0)))),
            new AprilTag(5, new Pose3d(9.27608, 1.914906, 1.867916, new Rotation3d(new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074, 0.0)))),
            new AprilTag(6, new Pose3d(13.474446, 3.306318, 0.308102, new Rotation3d(new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(7, new Pose3d(13.890498, 4.0259, 0.308102, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
            new AprilTag(8, new Pose3d(13.474446, 4.745482, 0.308102, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(9, new Pose3d(12.643358, 4.745482, 0.308102, new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
            new AprilTag(10, new Pose3d(12.227306, 4.0259, 0.308102, new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
            new AprilTag(11, new Pose3d(12.643358, 3.306318, 0.308102, new Rotation3d(new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387)))),
            new AprilTag(12, new Pose3d(0.851154, 0.65532, 1.4859, new Rotation3d(new Quaternion(0.8910065241883679, 0.0, 0.0, 0.45399049973954675)))),
            new AprilTag(13, new Pose3d(0.851154, 7.39648, 1.4859, new Rotation3d(new Quaternion(-0.8910065241883678, -0.0, 0.0, 0.45399049973954686)))),
            new AprilTag(14, new Pose3d(8.272272, 6.137656, 1.867916, new Rotation3d(new Quaternion(5.914589856893349e-17, -0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683)))),
            new AprilTag(15, new Pose3d(8.272272, 1.914906, 1.867916, new Rotation3d(new Quaternion(5.914589856893349e-17, -0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683)))),
            new AprilTag(16, new Pose3d(5.987542, -0.00381, 1.30175, new Rotation3d(new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
            new AprilTag(17, new Pose3d(4.073906, 3.306318, 0.308102, new Rotation3d(new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387)))),
            new AprilTag(18, new Pose3d(3.6576, 4.0259, 0.308102, new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
            new AprilTag(19, new Pose3d(4.073906, 4.745482, 0.308102, new Rotation3d(new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
            new AprilTag(20, new Pose3d(4.90474, 4.745482, 0.308102, new Rotation3d(new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
            new AprilTag(21, new Pose3d(5.321046, 4.0259, 0.308102, new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
            new AprilTag(22, new Pose3d(4.90474, 3.306318, 0.308102, new Rotation3d(new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994))))
    );
}