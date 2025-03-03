import math
import time
import ntcore
import config
import constants
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Translation2d, Translation3d

from subsystem import Drivetrain
from units.SI import seconds
from wpilib import Timer, RobotState, TimedRobot, DriverStation

from photonlibpy.estimatedRobotPose import EstimatedRobotPose
from sensors import PhotonController

class FieldOdometry:
    def __init__(
            self, drivetrain: Drivetrain, cam_controller: PhotonController | None, field_width: float = constants.field_width, field_length: float = constants.field_length
    ):
        self.drivetrain: Drivetrain = drivetrain
        self.table = ntcore.NetworkTableInstance.getDefault().getTable("Odometry")
        self.last_update_time: seconds | None = None
        self.min_update_wait_time: seconds = 0.05

        self.field_length = field_length
        self.field_width = field_width

        self.vision_on = True

        self.cam_controller = cam_controller

        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("Odometry")

    def enable(self):
        self.vision_on = True

    def disable(self):
        self.vision_on = False

    def update_from_internal(self):
        self.drivetrain.odometry_estimator.updateWithTime(
            Timer.getFPGATimestamp(),
            self.drivetrain.get_heading(),
            self.drivetrain.node_positions
        )

    def pose_within_field(self, pose: Pose2d):
        x_within = (0 + constants.drivetrain_length_with_bumpers/2) <= pose.X() <= (self.field_length - constants.drivetrain_length_with_bumpers/2)
        y_within = (0 + constants.drivetrain_length_with_bumpers/2) <= pose.Y() <= (self.field_width - constants.drivetrain_length_with_bumpers/2)
        return x_within and y_within

    def keep_pose_in_field(self):
        pose = self.drivetrain.odometry_estimator.getEstimatedPosition()
        
        new_x = max((0 + constants.drivetrain_length_with_bumpers/2), min(pose.X(), (self.field_length - constants.drivetrain_length_with_bumpers/2)))
        new_y = max((0 + constants.drivetrain_length_with_bumpers/2), min(pose.Y(), (self.field_width - constants.drivetrain_length_with_bumpers/2)))
        new_pose = Pose2d(new_x, new_y, pose.rotation())
    
        self.drivetrain.reset_odometry(new_pose)

    def add_vision_measure(self, estimated_pose: EstimatedRobotPose):
        pose = estimated_pose.estimatedPose.toPose2d()
        vision_time = estimated_pose.timestampSeconds
        tags = estimated_pose.targetsUsed
        tag_count = len(tags)
        tag_ids = [tag.fiducialId for tag in tags]
        primary_id = tag_ids[0]
        distance_to_target = tags[0].bestCameraToTarget.translation().toTranslation2d().distance(Translation2d(0, 0))
        
        std_dev = 2
        self.nt.putNumber("Distance to target", distance_to_target)

        # if not self.pose_within_field(pose):
        #     return
        
        if tag_count == 0:
            return
        
        if tag_count == 1:
            if distance_to_target > config.odometry_tag_distance_threshold:
                return
            if ((6 <= primary_id <= 11) | (17 <= primary_id <= 22)) & (distance_to_target <= 1.5):
                std_dev = 0.25
                if distance_to_target <= 0.75:
                    std_dev = 0.1
                    if DriverStation.isTeleop():
                        self.drivetrain.odometry_estimator.addVisionMeasurement(pose, vision_time, [std_dev, std_dev, std_dev])
                        return

        if tag_count >= 2:
            std_dev = 0.7

            if ((6 <= primary_id <= 11) | (17 <= primary_id <= 22)) & (distance_to_target <= 0.5):
                std_dev = 0.5
                if distance_to_target <= 0.25:
                    std_dev = 0.25
        

        self.drivetrain.odometry_estimator.addVisionMeasurement(Pose2d(pose.X(), pose.Y(), self.drivetrain.get_heading()), vision_time, [std_dev, std_dev, 50])
        # self.drivetrain.odometry_estimator.addVisionMeasurement(pose, vision_time, [std_dev, std_dev, std_dev])

    def getPose(self) -> Pose2d:
        """
        Returns the robot's pose relative to the field.
        :return: Robot pose.
        :rtype: Pose2d
        """
        est_pose = self.drivetrain.odometry_estimator.getEstimatedPosition()
        # if not self.vision_on or TimedRobot.isSimulation():
        #     est_pose = self.drivetrain.odometry.getPose()
        # else:
        #     self.drivetrain.odometry.resetPosition(
        #         self.drivetrain.get_heading(),
        #         self.drivetrain.node_positions,
        #         est_pose
        #     )
        return est_pose

    def get_vision_poses(self) -> list[EstimatedRobotPose] | None:
        if self.cam_controller is None:
            return None
        return self.cam_controller.get_results()

    def update(self) -> Pose2d:
        self.update_from_internal()

        # if not self.pose_within_field(self.getPose()):
        #     self.keep_pose_in_field()

        if not self.vision_on:
            return self.getPose()
        
        vision_robot_pose_list = self.get_vision_poses()

        if vision_robot_pose_list is None:
            return self.getPose()

        for vision_pose in vision_robot_pose_list:
            if vision_pose is None:
                continue

            self.add_vision_measure(vision_pose)

        return self.getPose()