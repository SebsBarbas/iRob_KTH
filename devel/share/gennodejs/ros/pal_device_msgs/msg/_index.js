
"use strict";

let BatteryState = require('./BatteryState.js');
let Bumper = require('./Bumper.js');
let LedBlinkParams = require('./LedBlinkParams.js');
let LedDataArrayParams = require('./LedDataArrayParams.js');
let LedEffectParams = require('./LedEffectParams.js');
let LedEffectViaTopicParams = require('./LedEffectViaTopicParams.js');
let LedFadeParams = require('./LedFadeParams.js');
let LedFixedColorParams = require('./LedFixedColorParams.js');
let LedFlowParams = require('./LedFlowParams.js');
let LedGroup = require('./LedGroup.js');
let LedPreProgrammedParams = require('./LedPreProgrammedParams.js');
let LedProgressParams = require('./LedProgressParams.js');
let LedRainbowParams = require('./LedRainbowParams.js');
let DoTimedLedEffectAction = require('./DoTimedLedEffectAction.js');
let DoTimedLedEffectGoal = require('./DoTimedLedEffectGoal.js');
let DoTimedLedEffectActionGoal = require('./DoTimedLedEffectActionGoal.js');
let DoTimedLedEffectResult = require('./DoTimedLedEffectResult.js');
let DoTimedLedEffectActionResult = require('./DoTimedLedEffectActionResult.js');
let DoTimedLedEffectFeedback = require('./DoTimedLedEffectFeedback.js');
let DoTimedLedEffectActionFeedback = require('./DoTimedLedEffectActionFeedback.js');

module.exports = {
  BatteryState: BatteryState,
  Bumper: Bumper,
  LedBlinkParams: LedBlinkParams,
  LedDataArrayParams: LedDataArrayParams,
  LedEffectParams: LedEffectParams,
  LedEffectViaTopicParams: LedEffectViaTopicParams,
  LedFadeParams: LedFadeParams,
  LedFixedColorParams: LedFixedColorParams,
  LedFlowParams: LedFlowParams,
  LedGroup: LedGroup,
  LedPreProgrammedParams: LedPreProgrammedParams,
  LedProgressParams: LedProgressParams,
  LedRainbowParams: LedRainbowParams,
  DoTimedLedEffectAction: DoTimedLedEffectAction,
  DoTimedLedEffectGoal: DoTimedLedEffectGoal,
  DoTimedLedEffectActionGoal: DoTimedLedEffectActionGoal,
  DoTimedLedEffectResult: DoTimedLedEffectResult,
  DoTimedLedEffectActionResult: DoTimedLedEffectActionResult,
  DoTimedLedEffectFeedback: DoTimedLedEffectFeedback,
  DoTimedLedEffectActionFeedback: DoTimedLedEffectActionFeedback,
};
