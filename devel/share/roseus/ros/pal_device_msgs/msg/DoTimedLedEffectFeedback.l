;; Auto-generated. Do not edit!


(when (boundp 'pal_device_msgs::DoTimedLedEffectFeedback)
  (if (not (find-package "PAL_DEVICE_MSGS"))
    (make-package "PAL_DEVICE_MSGS"))
  (shadow 'DoTimedLedEffectFeedback (find-package "PAL_DEVICE_MSGS")))
(unless (find-package "PAL_DEVICE_MSGS::DOTIMEDLEDEFFECTFEEDBACK")
  (make-package "PAL_DEVICE_MSGS::DOTIMEDLEDEFFECTFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude DoTimedLedEffectFeedback.msg.html


(defclass pal_device_msgs::DoTimedLedEffectFeedback
  :super ros::object
  :slots ())

(defmethod pal_device_msgs::DoTimedLedEffectFeedback
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get pal_device_msgs::DoTimedLedEffectFeedback :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get pal_device_msgs::DoTimedLedEffectFeedback :datatype-) "pal_device_msgs/DoTimedLedEffectFeedback")
(setf (get pal_device_msgs::DoTimedLedEffectFeedback :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======


")



(provide :pal_device_msgs/DoTimedLedEffectFeedback "d41d8cd98f00b204e9800998ecf8427e")

