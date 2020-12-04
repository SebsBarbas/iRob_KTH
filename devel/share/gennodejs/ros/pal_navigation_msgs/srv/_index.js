
"use strict";

let Acknowledgment = require('./Acknowledgment.js')
let ChangeBuilding = require('./ChangeBuilding.js')
let DisableEmergency = require('./DisableEmergency.js')
let FinalApproachPose = require('./FinalApproachPose.js')
let GetMapConfiguration = require('./GetMapConfiguration.js')
let GetNodes = require('./GetNodes.js')
let GetPOI = require('./GetPOI.js')
let GetSubMap = require('./GetSubMap.js')
let ListMaps = require('./ListMaps.js')
let RenameMap = require('./RenameMap.js')
let SafetyZone = require('./SafetyZone.js')
let SaveMap = require('./SaveMap.js')
let SetMapConfiguration = require('./SetMapConfiguration.js')
let SetPOI = require('./SetPOI.js')
let SetSubMapFloor = require('./SetSubMapFloor.js')
let VisualLocRecognize = require('./VisualLocRecognize.js')

module.exports = {
  Acknowledgment: Acknowledgment,
  ChangeBuilding: ChangeBuilding,
  DisableEmergency: DisableEmergency,
  FinalApproachPose: FinalApproachPose,
  GetMapConfiguration: GetMapConfiguration,
  GetNodes: GetNodes,
  GetPOI: GetPOI,
  GetSubMap: GetSubMap,
  ListMaps: ListMaps,
  RenameMap: RenameMap,
  SafetyZone: SafetyZone,
  SaveMap: SaveMap,
  SetMapConfiguration: SetMapConfiguration,
  SetPOI: SetPOI,
  SetSubMapFloor: SetSubMapFloor,
  VisualLocRecognize: VisualLocRecognize,
};
