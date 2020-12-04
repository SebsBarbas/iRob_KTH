; Auto-generated. Do not edit!


(cl:in-package pal_interaction_msgs-msg)


;//! \htmlinclude ASRFileActionResult.msg.html

(cl:defclass <ASRFileActionResult> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (status
    :reader status
    :initarg :status
    :type actionlib_msgs-msg:GoalStatus
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalStatus))
   (result
    :reader result
    :initarg :result
    :type pal_interaction_msgs-msg:ASRFileResult
    :initform (cl:make-instance 'pal_interaction_msgs-msg:ASRFileResult)))
)

(cl:defclass ASRFileActionResult (<ASRFileActionResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ASRFileActionResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ASRFileActionResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_interaction_msgs-msg:<ASRFileActionResult> is deprecated: use pal_interaction_msgs-msg:ASRFileActionResult instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ASRFileActionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_interaction_msgs-msg:header-val is deprecated.  Use pal_interaction_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ASRFileActionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_interaction_msgs-msg:status-val is deprecated.  Use pal_interaction_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <ASRFileActionResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_interaction_msgs-msg:result-val is deprecated.  Use pal_interaction_msgs-msg:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ASRFileActionResult>) ostream)
  "Serializes a message object of type '<ASRFileActionResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'result) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ASRFileActionResult>) istream)
  "Deserializes a message object of type '<ASRFileActionResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'result) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ASRFileActionResult>)))
  "Returns string type for a message object of type '<ASRFileActionResult>"
  "pal_interaction_msgs/ASRFileActionResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ASRFileActionResult)))
  "Returns string type for a message object of type 'ASRFileActionResult"
  "pal_interaction_msgs/ASRFileActionResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ASRFileActionResult>)))
  "Returns md5sum for a message object of type '<ASRFileActionResult>"
  "d81072169f81a7734640dee69c1517f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ASRFileActionResult)))
  "Returns md5sum for a message object of type 'ASRFileActionResult"
  "d81072169f81a7734640dee69c1517f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ASRFileActionResult>)))
  "Returns full string definition for message of type '<ASRFileActionResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ASRFileResult result~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: pal_interaction_msgs/ASRFileResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%##result definition~%# same path as specified in goal variable file~%string file~%# error/warning messages ~%string msg~%# vector of results~%asrresult[] recognised_utterances~%~%================================================================================~%MSG: pal_interaction_msgs/asrresult~%## Message that containes the recognized utterance.~%## Confidence values~%int8 CONFIDENCE_UNDEFINED = -1~%int8 CONFIDENCE_POOR = 1~%int8 CONFIDENCE_LOW  = 2~%int8 CONFIDENCE_GOOD = 3~%int8 CONFIDENCE_MAX  = 4~%~%# ASR result messages used by RosRecognizerServer~%~%# text recognized~%string text~%~%# confidence with values from POOR to MAX~%int8 confidence~%~%# start and end of the recognizer uterance.~%time start~%time end~%~%# list of recognized tags~%# key value pairs of strings extracted from the text~%# given the action tags placed in the grammar.~%actiontag[] tags~%~%================================================================================~%MSG: pal_interaction_msgs/actiontag~%# Action tag contaings the key/value information genertated by parsing the recognised text with a JSGF grammar ~%~%string key~%string value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ASRFileActionResult)))
  "Returns full string definition for message of type 'ASRFileActionResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%ASRFileResult result~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: pal_interaction_msgs/ASRFileResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%##result definition~%# same path as specified in goal variable file~%string file~%# error/warning messages ~%string msg~%# vector of results~%asrresult[] recognised_utterances~%~%================================================================================~%MSG: pal_interaction_msgs/asrresult~%## Message that containes the recognized utterance.~%## Confidence values~%int8 CONFIDENCE_UNDEFINED = -1~%int8 CONFIDENCE_POOR = 1~%int8 CONFIDENCE_LOW  = 2~%int8 CONFIDENCE_GOOD = 3~%int8 CONFIDENCE_MAX  = 4~%~%# ASR result messages used by RosRecognizerServer~%~%# text recognized~%string text~%~%# confidence with values from POOR to MAX~%int8 confidence~%~%# start and end of the recognizer uterance.~%time start~%time end~%~%# list of recognized tags~%# key value pairs of strings extracted from the text~%# given the action tags placed in the grammar.~%actiontag[] tags~%~%================================================================================~%MSG: pal_interaction_msgs/actiontag~%# Action tag contaings the key/value information genertated by parsing the recognised text with a JSGF grammar ~%~%string key~%string value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ASRFileActionResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'result))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ASRFileActionResult>))
  "Converts a ROS message object to a list"
  (cl:list 'ASRFileActionResult
    (cl:cons ':header (header msg))
    (cl:cons ':status (status msg))
    (cl:cons ':result (result msg))
))