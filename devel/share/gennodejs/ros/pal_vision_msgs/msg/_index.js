
"use strict";

let DetectedObject = require('./DetectedObject.js');
let DetectedPerson = require('./DetectedPerson.js');
let FaceDetection = require('./FaceDetection.js');
let FaceDetections = require('./FaceDetections.js');
let FollowMeResponse = require('./FollowMeResponse.js');
let Gesture = require('./Gesture.js');
let HeadPanTilt = require('./HeadPanTilt.js');
let HogDetection = require('./HogDetection.js');
let HogDetections = require('./HogDetections.js');
let LegDetections = require('./LegDetections.js');
let Rectangle = require('./Rectangle.js');
let FaceRecognitionAction = require('./FaceRecognitionAction.js');
let FaceRecognitionGoal = require('./FaceRecognitionGoal.js');
let FaceRecognitionActionGoal = require('./FaceRecognitionActionGoal.js');
let FaceRecognitionResult = require('./FaceRecognitionResult.js');
let FaceRecognitionActionResult = require('./FaceRecognitionActionResult.js');
let FaceRecognitionFeedback = require('./FaceRecognitionFeedback.js');
let FaceRecognitionActionFeedback = require('./FaceRecognitionActionFeedback.js');

module.exports = {
  DetectedObject: DetectedObject,
  DetectedPerson: DetectedPerson,
  FaceDetection: FaceDetection,
  FaceDetections: FaceDetections,
  FollowMeResponse: FollowMeResponse,
  Gesture: Gesture,
  HeadPanTilt: HeadPanTilt,
  HogDetection: HogDetection,
  HogDetections: HogDetections,
  LegDetections: LegDetections,
  Rectangle: Rectangle,
  FaceRecognitionAction: FaceRecognitionAction,
  FaceRecognitionGoal: FaceRecognitionGoal,
  FaceRecognitionActionGoal: FaceRecognitionActionGoal,
  FaceRecognitionResult: FaceRecognitionResult,
  FaceRecognitionActionResult: FaceRecognitionActionResult,
  FaceRecognitionFeedback: FaceRecognitionFeedback,
  FaceRecognitionActionFeedback: FaceRecognitionActionFeedback,
};
