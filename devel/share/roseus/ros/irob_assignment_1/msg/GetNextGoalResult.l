;; Auto-generated. Do not edit!


(when (boundp 'irob_assignment_1::GetNextGoalResult)
  (if (not (find-package "IROB_ASSIGNMENT_1"))
    (make-package "IROB_ASSIGNMENT_1"))
  (shadow 'GetNextGoalResult (find-package "IROB_ASSIGNMENT_1")))
(unless (find-package "IROB_ASSIGNMENT_1::GETNEXTGOALRESULT")
  (make-package "IROB_ASSIGNMENT_1::GETNEXTGOALRESULT"))

(in-package "ROS")
;;//! \htmlinclude GetNextGoalResult.msg.html
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass irob_assignment_1::GetNextGoalResult
  :super ros::object
  :slots (_gain _path ))

(defmethod irob_assignment_1::GetNextGoalResult
  (:init
   (&key
    ((:gain __gain) 0.0)
    ((:path __path) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _gain (float __gain))
   (setq _path __path)
   self)
  (:gain
   (&optional __gain)
   (if __gain (setq _gain __gain)) _gain)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; float64 _gain
    8
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _gain
       (sys::poke _gain (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; nav_msgs/Path _path
       (send _path :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _gain
     (setq _gain (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;;
   self)
  )

(setf (get irob_assignment_1::GetNextGoalResult :md5sum-) "3fd937c20feb920775439a64b763c7e5")
(setf (get irob_assignment_1::GetNextGoalResult :datatype-) "irob_assignment_1/GetNextGoalResult")
(setf (get irob_assignment_1::GetNextGoalResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# Result definition
float64 gain
nav_msgs/Path path

================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :irob_assignment_1/GetNextGoalResult "3fd937c20feb920775439a64b763c7e5")

