;; Auto-generated. Do not edit!


(when (boundp 'pal_device_msgs::DoTimedLedEffectResult)
  (if (not (find-package "PAL_DEVICE_MSGS"))
    (make-package "PAL_DEVICE_MSGS"))
  (shadow 'DoTimedLedEffectResult (find-package "PAL_DEVICE_MSGS")))
(unless (find-package "PAL_DEVICE_MSGS::DOTIMEDLEDEFFECTRESULT")
  (make-package "PAL_DEVICE_MSGS::DOTIMEDLEDEFFECTRESULT"))

(in-package "ROS")
;;//! \htmlinclude DoTimedLedEffectResult.msg.html


(defclass pal_device_msgs::DoTimedLedEffectResult
  :super ros::object
  :slots ())

(defmethod pal_device_msgs::DoTimedLedEffectResult
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

(setf (get pal_device_msgs::DoTimedLedEffectResult :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get pal_device_msgs::DoTimedLedEffectResult :datatype-) "pal_device_msgs/DoTimedLedEffectResult")
(setf (get pal_device_msgs::DoTimedLedEffectResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

")



(provide :pal_device_msgs/DoTimedLedEffectResult "d41d8cd98f00b204e9800998ecf8427e")


