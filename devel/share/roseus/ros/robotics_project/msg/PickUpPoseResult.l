;; Auto-generated. Do not edit!


(when (boundp 'robotics_project::PickUpPoseResult)
  (if (not (find-package "ROBOTICS_PROJECT"))
    (make-package "ROBOTICS_PROJECT"))
  (shadow 'PickUpPoseResult (find-package "ROBOTICS_PROJECT")))
(unless (find-package "ROBOTICS_PROJECT::PICKUPPOSERESULT")
  (make-package "ROBOTICS_PROJECT::PICKUPPOSERESULT"))

(in-package "ROS")
;;//! \htmlinclude PickUpPoseResult.msg.html


(defclass robotics_project::PickUpPoseResult
  :super ros::object
  :slots (_error_code ))

(defmethod robotics_project::PickUpPoseResult
  (:init
   (&key
    ((:error_code __error_code) 0)
    )
   (send-super :init)
   (setq _error_code (round __error_code))
   self)
  (:error_code
   (&optional __error_code)
   (if __error_code (setq _error_code __error_code)) _error_code)
  (:serialization-length
   ()
   (+
    ;; int32 _error_code
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _error_code
       (write-long _error_code s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _error_code
     (setq _error_code (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotics_project::PickUpPoseResult :md5sum-) "ea324a22c787839f822b9a025bc2c6fe")
(setf (get robotics_project::PickUpPoseResult :datatype-) "robotics_project/PickUpPoseResult")
(setf (get robotics_project::PickUpPoseResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#result definition
# Comes from moveit_msgs/MoveItErrorCodes
int32 error_code

")



(provide :robotics_project/PickUpPoseResult "ea324a22c787839f822b9a025bc2c6fe")


