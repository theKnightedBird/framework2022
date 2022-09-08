package frc.team449.control.auto.TrajFollow

import com.pathplanner.lib.PathPlanner
import frc.team449.robot2022.drive.DriveConstants

object trajList {

  val testAutoPath =
    PathPlanner.loadPath(
      "example",
      DriveConstants.MAX_LINEAR_SPEED,
      DriveConstants.MAX_LINEAR_ACCEL
    )

}