; Auto-generated. Do not edit!


(cl:in-package tf_lookup-msg)


;//! \htmlinclude TfLookupFeedback.msg.html

(cl:defclass <TfLookupFeedback> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass TfLookupFeedback (<TfLookupFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TfLookupFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TfLookupFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tf_lookup-msg:<TfLookupFeedback> is deprecated: use tf_lookup-msg:TfLookupFeedback instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TfLookupFeedback>) ostream)
  "Serializes a message object of type '<TfLookupFeedback>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TfLookupFeedback>) istream)
  "Deserializes a message object of type '<TfLookupFeedback>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TfLookupFeedback>)))
  "Returns string type for a message object of type '<TfLookupFeedback>"
  "tf_lookup/TfLookupFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TfLookupFeedback)))
  "Returns string type for a message object of type 'TfLookupFeedback"
  "tf_lookup/TfLookupFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TfLookupFeedback>)))
  "Returns md5sum for a message object of type '<TfLookupFeedback>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TfLookupFeedback)))
  "Returns md5sum for a message object of type 'TfLookupFeedback"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TfLookupFeedback>)))
  "Returns full string definition for message of type '<TfLookupFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TfLookupFeedback)))
  "Returns full string definition for message of type 'TfLookupFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TfLookupFeedback>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TfLookupFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'TfLookupFeedback
))