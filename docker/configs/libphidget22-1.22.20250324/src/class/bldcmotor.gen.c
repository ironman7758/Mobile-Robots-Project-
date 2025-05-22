/* Generated: Mon Mar 24 2025 13:18:57 GMT-0600 (Mountain Daylight Time) */

#include "device/vintdevice.h"
static void CCONV PhidgetBLDCMotor_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetBLDCMotor_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetBLDCMotor_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetBLDCMotor_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetBLDCMotor_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetBLDCMotor {
	struct _PhidgetChannel phid;
	int64_t positionOffset;
	double acceleration;
	double minAcceleration;
	double maxAcceleration;
	double activeCurrentLimit;
	double brakingStrength;
	double minBrakingStrength;
	double maxBrakingStrength;
	double currentLimit;
	double minCurrentLimit;
	double maxCurrentLimit;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	Phidget_DriveMode driveMode;
	int failsafeBrakingEnabled;
	double failsafeCurrentLimit;
	uint32_t minFailsafeTime;
	uint32_t maxFailsafeTime;
	double inductance;
	double minInductance;
	double maxInductance;
	int64_t position;
	int64_t minPosition;
	int64_t maxPosition;
	double rescaleFactor;
	double stallVelocity;
	double minStallVelocity;
	double maxStallVelocity;
	double surgeCurrentLimit;
	double minSurgeCurrentLimit;
	double maxSurgeCurrentLimit;
	double targetBrakingStrength;
	double targetVelocity;
	double velocity;
	double minVelocity;
	double maxVelocity;
	volatile PhidgetBLDCMotor_OnBrakingStrengthChangeCallback BrakingStrengthChange;
	volatile void *BrakingStrengthChangeCtx;
	volatile PhidgetBLDCMotor_OnPositionChangeCallback PositionChange;
	volatile void *PositionChangeCtx;
	volatile PhidgetBLDCMotor_OnVelocityUpdateCallback VelocityUpdate;
	volatile void *VelocityUpdateCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetBLDCMotorHandle ch;
	int nsversion;

	ch = (PhidgetBLDCMotorHandle)phid;

	nsversion = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (nsversion != 4) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 4 - functionality may be limited.", phid, nsversion);
	}

	if(hasBridgePacketByName(bp, "positionOffset"))
		ch->positionOffset = getBridgePacketInt64ByName(bp, "positionOffset");
	if (hasBridgePacketByName(bp, "acceleration"))
		ch->acceleration = getBridgePacketDoubleByName(bp, "acceleration");
	if (hasBridgePacketByName(bp, "minAcceleration"))
		ch->minAcceleration = getBridgePacketDoubleByName(bp, "minAcceleration");
	if (hasBridgePacketByName(bp, "maxAcceleration"))
		ch->maxAcceleration = getBridgePacketDoubleByName(bp, "maxAcceleration");
	if (hasBridgePacketByName(bp, "activeCurrentLimit"))
		ch->activeCurrentLimit = getBridgePacketDoubleByName(bp, "activeCurrentLimit");
	if (hasBridgePacketByName(bp, "brakingStrength"))
		ch->brakingStrength = getBridgePacketDoubleByName(bp, "brakingStrength");
	if (hasBridgePacketByName(bp, "minBrakingStrength"))
		ch->minBrakingStrength = getBridgePacketDoubleByName(bp, "minBrakingStrength");
	if (hasBridgePacketByName(bp, "maxBrakingStrength"))
		ch->maxBrakingStrength = getBridgePacketDoubleByName(bp, "maxBrakingStrength");
	if (hasBridgePacketByName(bp, "currentLimit"))
		ch->currentLimit = getBridgePacketDoubleByName(bp, "currentLimit");
	if (hasBridgePacketByName(bp, "minCurrentLimit"))
		ch->minCurrentLimit = getBridgePacketDoubleByName(bp, "minCurrentLimit");
	if (hasBridgePacketByName(bp, "maxCurrentLimit"))
		ch->maxCurrentLimit = getBridgePacketDoubleByName(bp, "maxCurrentLimit");
	if (hasBridgePacketByName(bp, "minDataInterval"))
		ch->minDataInterval = getBridgePacketUInt32ByName(bp, "minDataInterval");
	if (hasBridgePacketByName(bp, "maxDataInterval"))
		ch->maxDataInterval = getBridgePacketUInt32ByName(bp, "maxDataInterval");
	if (hasBridgePacketByName(bp, "dataIntervalDbl"))
		ch->dataInterval = getBridgePacketDoubleByName(bp, "dataIntervalDbl");
	else if(hasBridgePacketByName(bp, "dataInterval"))
		ch->dataInterval = (double)getBridgePacketUInt32ByName(bp, "dataInterval");
	if (hasBridgePacketByName(bp, "minDataRate"))
		ch->minDataRate = getBridgePacketDoubleByName(bp, "minDataRate");
	else if(hasBridgePacketByName(bp, "maxDataInterval"))
		ch->minDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "maxDataInterval"));
	if (hasBridgePacketByName(bp, "maxDataRate"))
		ch->maxDataRate = getBridgePacketDoubleByName(bp, "maxDataRate");
	else if(hasBridgePacketByName(bp, "minDataInterval"))
		ch->maxDataRate = (double)(1000.0 / getBridgePacketUInt32ByName(bp, "minDataInterval"));
	if (hasBridgePacketByName(bp, "driveMode"))
		ch->driveMode = getBridgePacketInt32ByName(bp, "driveMode");
	if (hasBridgePacketByName(bp, "failsafeBrakingEnabled"))
		ch->failsafeBrakingEnabled = getBridgePacketInt32ByName(bp, "failsafeBrakingEnabled");
	if (hasBridgePacketByName(bp, "failsafeCurrentLimit"))
		ch->failsafeCurrentLimit = getBridgePacketDoubleByName(bp, "failsafeCurrentLimit");
	if (hasBridgePacketByName(bp, "minFailsafeTime"))
		ch->minFailsafeTime = getBridgePacketUInt32ByName(bp, "minFailsafeTime");
	if (hasBridgePacketByName(bp, "maxFailsafeTime"))
		ch->maxFailsafeTime = getBridgePacketUInt32ByName(bp, "maxFailsafeTime");
	if (hasBridgePacketByName(bp, "inductance"))
		ch->inductance = getBridgePacketDoubleByName(bp, "inductance");
	if (hasBridgePacketByName(bp, "minInductance"))
		ch->minInductance = getBridgePacketDoubleByName(bp, "minInductance");
	if (hasBridgePacketByName(bp, "maxInductance"))
		ch->maxInductance = getBridgePacketDoubleByName(bp, "maxInductance");
	if (hasBridgePacketByName(bp, "position"))
		ch->position = getBridgePacketInt64ByName(bp, "position");
	if (hasBridgePacketByName(bp, "minPosition"))
		ch->minPosition = getBridgePacketInt64ByName(bp, "minPosition");
	if (hasBridgePacketByName(bp, "maxPosition"))
		ch->maxPosition = getBridgePacketInt64ByName(bp, "maxPosition");
	if (hasBridgePacketByName(bp, "rescaleFactor"))
		ch->rescaleFactor = getBridgePacketDoubleByName(bp, "rescaleFactor");
	if (hasBridgePacketByName(bp, "stallVelocity"))
		ch->stallVelocity = getBridgePacketDoubleByName(bp, "stallVelocity");
	if (hasBridgePacketByName(bp, "minStallVelocity"))
		ch->minStallVelocity = getBridgePacketDoubleByName(bp, "minStallVelocity");
	if (hasBridgePacketByName(bp, "maxStallVelocity"))
		ch->maxStallVelocity = getBridgePacketDoubleByName(bp, "maxStallVelocity");
	if (hasBridgePacketByName(bp, "surgeCurrentLimit"))
		ch->surgeCurrentLimit = getBridgePacketDoubleByName(bp, "surgeCurrentLimit");
	if (hasBridgePacketByName(bp, "minSurgeCurrentLimit"))
		ch->minSurgeCurrentLimit = getBridgePacketDoubleByName(bp, "minSurgeCurrentLimit");
	if (hasBridgePacketByName(bp, "maxSurgeCurrentLimit"))
		ch->maxSurgeCurrentLimit = getBridgePacketDoubleByName(bp, "maxSurgeCurrentLimit");
	if (hasBridgePacketByName(bp, "targetBrakingStrength"))
		ch->targetBrakingStrength = getBridgePacketDoubleByName(bp, "targetBrakingStrength");
	if (hasBridgePacketByName(bp, "targetVelocity"))
		ch->targetVelocity = getBridgePacketDoubleByName(bp, "targetVelocity");
	if (hasBridgePacketByName(bp, "velocity"))
		ch->velocity = getBridgePacketDoubleByName(bp, "velocity");
	if (hasBridgePacketByName(bp, "minVelocity"))
		ch->minVelocity = getBridgePacketDoubleByName(bp, "minVelocity");
	if (hasBridgePacketByName(bp, "maxVelocity"))
		ch->maxVelocity = getBridgePacketDoubleByName(bp, "maxVelocity");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetBLDCMotorHandle ch;

	ch = (PhidgetBLDCMotorHandle)phid;

	return (createBridgePacket(bp, BP_SETSTATUS, 41, "_class_version_=%u"
	  ",positionOffset=%l"
	  ",acceleration=%g"
	  ",minAcceleration=%g"
	  ",maxAcceleration=%g"
	  ",activeCurrentLimit=%g"
	  ",brakingStrength=%g"
	  ",minBrakingStrength=%g"
	  ",maxBrakingStrength=%g"
	  ",currentLimit=%g"
	  ",minCurrentLimit=%g"
	  ",maxCurrentLimit=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",driveMode=%d"
	  ",failsafeBrakingEnabled=%d"
	  ",failsafeCurrentLimit=%g"
	  ",minFailsafeTime=%u"
	  ",maxFailsafeTime=%u"
	  ",inductance=%g"
	  ",minInductance=%g"
	  ",maxInductance=%g"
	  ",position=%l"
	  ",minPosition=%l"
	  ",maxPosition=%l"
	  ",rescaleFactor=%g"
	  ",stallVelocity=%g"
	  ",minStallVelocity=%g"
	  ",maxStallVelocity=%g"
	  ",surgeCurrentLimit=%g"
	  ",minSurgeCurrentLimit=%g"
	  ",maxSurgeCurrentLimit=%g"
	  ",targetBrakingStrength=%g"
	  ",targetVelocity=%g"
	  ",velocity=%g"
	  ",minVelocity=%g"
	  ",maxVelocity=%g"
	  ,4 /* class version */
	  ,ch->positionOffset
	  ,ch->acceleration
	  ,ch->minAcceleration
	  ,ch->maxAcceleration
	  ,ch->activeCurrentLimit
	  ,ch->brakingStrength
	  ,ch->minBrakingStrength
	  ,ch->maxBrakingStrength
	  ,ch->currentLimit
	  ,ch->minCurrentLimit
	  ,ch->maxCurrentLimit
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->driveMode
	  ,ch->failsafeBrakingEnabled
	  ,ch->failsafeCurrentLimit
	  ,ch->minFailsafeTime
	  ,ch->maxFailsafeTime
	  ,ch->inductance
	  ,ch->minInductance
	  ,ch->maxInductance
	  ,ch->position
	  ,ch->minPosition
	  ,ch->maxPosition
	  ,ch->rescaleFactor
	  ,ch->stallVelocity
	  ,ch->minStallVelocity
	  ,ch->maxStallVelocity
	  ,ch->surgeCurrentLimit
	  ,ch->minSurgeCurrentLimit
	  ,ch->maxSurgeCurrentLimit
	  ,ch->targetBrakingStrength
	  ,ch->targetVelocity
	  ,ch->velocity
	  ,ch->minVelocity
	  ,ch->maxVelocity
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetBLDCMotorHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetBLDCMotorHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_SETFAILSAFETIME:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_FAILSAFERESET:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_SETACCELERATION:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minAcceleration,
		  ch->maxAcceleration);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->acceleration = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Acceleration");
		}
		break;
	case BP_SETCURRENTLIMIT:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minCurrentLimit,
		  ch->maxCurrentLimit);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->currentLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "CurrentLimit");
		}
		break;
	case BP_SETDATAINTERVAL:
		if (bp->entrycnt > 1)
			TESTRANGE_IOP(bp->iop, "%lf", round_double((1000.0 / getBridgePacketDouble(bp, 1)), 4),
			  ch->minDataRate, ch->maxDataRate);
		else
			TESTRANGE_IOP(bp->iop, "%"PRIu32, getBridgePacketUInt32(bp, 0), ch->minDataInterval,
			  ch->maxDataInterval);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		if (bp->entrycnt > 1)
			ch->dataInterval = getBridgePacketDouble(bp, 1);
		else
			ch->dataInterval = (double)getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "DataInterval");
			FIRE_PROPERTYCHANGE(ch, "DataRate");
		}
		break;
	case BP_DRIVEMODE:
		if (!supportedDriveMode(phid, (Phidget_DriveMode)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified DriveMode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->driveMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "DriveMode");
		}
		break;
	case BP_SETFAILSAFEBRAKINGDUTYCYCLE:
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->failsafeBrakingEnabled = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "FailsafeBrakingEnabled");
		}
		break;
	case BP_SETFAILSAFECURRENTLIMIT:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->failsafeCurrentLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "FailsafeCurrentLimit");
		}
		break;
	case BP_SETINDUCTANCE:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minInductance,
		  ch->maxInductance);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->inductance = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Inductance");
		}
		break;
	case BP_SETSTALLVELOCITY:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minStallVelocity,
		  ch->maxStallVelocity);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->stallVelocity = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "StallVelocity");
		}
		break;
	case BP_SETSURGECURRENTLIMIT:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minSurgeCurrentLimit,
		  ch->maxSurgeCurrentLimit);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->surgeCurrentLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "SurgeCurrentLimit");
		}
		break;
	case BP_SETBRAKINGDUTYCYCLE:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->targetBrakingStrength = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "TargetBrakingStrength");
		}
		break;
	case BP_SETDUTYCYCLE:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->targetVelocity = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "TargetVelocity");
		}
		break;
	case BP_BRAKINGSTRENGTHCHANGE:
		ch->brakingStrength = getBridgePacketDouble(bp, 0);
		FIRECH(ch, PhidgetBLDCMotor, BrakingStrengthChange, ch->brakingStrength);
		break;
	case BP_DUTYCYCLECHANGE:
		ch->velocity = getBridgePacketDouble(bp, 0);
		FIRECH(ch, PhidgetBLDCMotor, VelocityUpdate, ch->velocity);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetBLDCMotorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetBLDCMotorHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->maxAcceleration = 100;
		ch->maxBrakingStrength = 1;
		ch->maxVelocity = 1;
		ch->maxPosition = 1000000000000000;
		ch->minVelocity = 0;
		ch->minAcceleration = 0.1;
		ch->minBrakingStrength = 0;
		ch->minPosition = -1000000000000000;
		ch->position = 0;
		ch->rescaleFactor = 1;
		ch->targetVelocity = 0;
		ch->velocity = PUNK_DBL;
		ch->brakingStrength = PUNK_DBL;
		ch->stallVelocity = 400;
		ch->minStallVelocity = 0;
		ch->maxStallVelocity = 2000;
		break;
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		ch->dataInterval = 250;
		ch->minDataInterval = 100;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 10;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->maxAcceleration = 100;
		ch->maxBrakingStrength = 1;
		ch->maxVelocity = 1;
		ch->maxPosition = 1000000000000000;
		ch->minVelocity = 0;
		ch->minAcceleration = 0.1;
		ch->minBrakingStrength = 0;
		ch->minPosition = -1000000000000000;
		ch->position = 0;
		ch->rescaleFactor = 1;
		ch->targetVelocity = 0;
		ch->velocity = PUNK_DBL;
		ch->brakingStrength = PUNK_DBL;
		ch->stallVelocity = 400;
		ch->minStallVelocity = 0;
		ch->maxStallVelocity = 2000;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		break;
	case PHIDCHUID_DCC1120_BLDCMOTOR_100:
		ch->dataInterval = 250;
		ch->minDataInterval = 10;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 100;
		ch->currentLimit = 5;
		ch->maxCurrentLimit = 50;
		ch->minCurrentLimit = 5;
		ch->surgeCurrentLimit = 5;
		ch->maxSurgeCurrentLimit = 75;
		ch->minSurgeCurrentLimit = 5;
		ch->activeCurrentLimit = PUNK_DBL;
		ch->acceleration = 1;
		ch->targetBrakingStrength = 0;
		ch->maxAcceleration = 1000;
		ch->maxBrakingStrength = 1;
		ch->maxVelocity = 1;
		ch->maxPosition = 1000000000000000;
		ch->minVelocity = 0;
		ch->minAcceleration = 0.1;
		ch->minBrakingStrength = 0;
		ch->minPosition = -1000000000000000;
		ch->position = 0;
		ch->rescaleFactor = 1;
		ch->targetVelocity = 0;
		ch->velocity = PUNK_DBL;
		ch->brakingStrength = PUNK_DBL;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		ch->inductance = PUNK_DBL;
		ch->maxInductance = 0.01;
		ch->minInductance = 0.0001;
		ch->driveMode = DRIVE_MODE_COAST;
		ch->failsafeCurrentLimit = 5;
		ch->failsafeBrakingEnabled = 0;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	ch->positionOffset = 0;

	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetBLDCMotorHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetBLDCMotorHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, 1, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETBRAKINGDUTYCYCLE, NULL, NULL, 1, "%g",
		  ch->targetBrakingStrength);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g", ch->targetVelocity);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSTALLVELOCITY, NULL, NULL, 1, "%g", ch->stallVelocity);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, 1, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETBRAKINGDUTYCYCLE, NULL, NULL, 1, "%g",
		  ch->targetBrakingStrength);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g", ch->targetVelocity);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSTALLVELOCITY, NULL, NULL, 1, "%g", ch->stallVelocity);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DCC1120_BLDCMOTOR_100:
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, 1, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g", ch->targetVelocity);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_DRIVEMODE, NULL, NULL, 1, "%d", ch->driveMode);
		if (ret != EPHIDGET_OK)
			break;
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {
	PhidgetBLDCMotorHandle ch;

	ch = (PhidgetBLDCMotorHandle)phid;

	if(ch->brakingStrength != PUNK_DBL)
		FIRECH(ch, PhidgetBLDCMotor, BrakingStrengthChange, ch->brakingStrength);
	if(ch->velocity != PUNK_DBL)
		FIRECH(ch, PhidgetBLDCMotor, VelocityUpdate, ch->velocity);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetBLDCMotorHandle ch;

	ch = (PhidgetBLDCMotorHandle)phid;

	if(ch->brakingStrength == PUNK_DBL)
		return (PFALSE);
	if(ch->velocity == PUNK_DBL)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetBLDCMotor));
}

static PhidgetReturnCode CCONV
_create(PhidgetBLDCMotorHandle *phidp) {

	CHANNELCREATE_BODY(BLDCMotor, PHIDCHCLASS_BLDCMOTOR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_delete(PhidgetBLDCMotorHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetBLDCMotor_enableFailsafe(PhidgetBLDCMotorHandle ch, uint32_t failsafeTime) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFAILSAFETIME, NULL, NULL, 1, "%u",
	  failsafeTime); 
}

API_PRETURN
PhidgetBLDCMotor_resetFailsafe(PhidgetBLDCMotorHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_FAILSAFERESET, NULL, NULL, 0, NULL); 
}

API_PRETURN
PhidgetBLDCMotor_setAcceleration(PhidgetBLDCMotorHandle ch, double acceleration) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETACCELERATION, NULL, NULL, 1, "%g",
	  acceleration));
}

API_PRETURN
PhidgetBLDCMotor_getAcceleration(PhidgetBLDCMotorHandle ch, double *acceleration) {

	TESTPTR_PR(ch);
	TESTPTR_PR(acceleration);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*acceleration = ch->acceleration;
	if (ch->acceleration == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinAcceleration(PhidgetBLDCMotorHandle ch, double *minAcceleration) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minAcceleration);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*minAcceleration = ch->minAcceleration;
	if (ch->minAcceleration == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxAcceleration(PhidgetBLDCMotorHandle ch, double *maxAcceleration) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxAcceleration);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*maxAcceleration = ch->maxAcceleration;
	if (ch->maxAcceleration == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getActiveCurrentLimit(PhidgetBLDCMotorHandle ch, double *activeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(activeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*activeCurrentLimit = ch->activeCurrentLimit;
	if (ch->activeCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getBrakingStrength(PhidgetBLDCMotorHandle ch, double *brakingStrength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(brakingStrength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*brakingStrength = ch->brakingStrength;
	if (ch->brakingStrength == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinBrakingStrength(PhidgetBLDCMotorHandle ch, double *minBrakingStrength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minBrakingStrength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*minBrakingStrength = ch->minBrakingStrength;
	if (ch->minBrakingStrength == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxBrakingStrength(PhidgetBLDCMotorHandle ch, double *maxBrakingStrength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxBrakingStrength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*maxBrakingStrength = ch->maxBrakingStrength;
	if (ch->maxBrakingStrength == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setCurrentLimit(PhidgetBLDCMotorHandle ch, double currentLimit) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCURRENTLIMIT, NULL, NULL, 1, "%g",
	  currentLimit));
}

API_PRETURN
PhidgetBLDCMotor_getCurrentLimit(PhidgetBLDCMotorHandle ch, double *currentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(currentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*currentLimit = ch->currentLimit;
	if (ch->currentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinCurrentLimit(PhidgetBLDCMotorHandle ch, double *minCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minCurrentLimit = ch->minCurrentLimit;
	if (ch->minCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxCurrentLimit(PhidgetBLDCMotorHandle ch, double *maxCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxCurrentLimit = ch->maxCurrentLimit;
	if (ch->maxCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setDataInterval(PhidgetBLDCMotorHandle ch, uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetBLDCMotor_getDataInterval(PhidgetBLDCMotorHandle ch, uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinDataInterval(PhidgetBLDCMotorHandle ch, uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxDataInterval(PhidgetBLDCMotorHandle ch, uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setDataRate(PhidgetBLDCMotorHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, 2, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetBLDCMotor_getDataRate(PhidgetBLDCMotorHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinDataRate(PhidgetBLDCMotorHandle ch, double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxDataRate(PhidgetBLDCMotorHandle ch, double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setDriveMode(PhidgetBLDCMotorHandle ch, Phidget_DriveMode driveMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_DRIVEMODE, NULL, NULL, 1, "%d",
	  driveMode));
}

API_PRETURN
PhidgetBLDCMotor_getDriveMode(PhidgetBLDCMotorHandle ch, Phidget_DriveMode *driveMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(driveMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*driveMode = ch->driveMode;
	if (ch->driveMode == (Phidget_DriveMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setFailsafeBrakingEnabled(PhidgetBLDCMotorHandle ch, int failsafeBrakingEnabled) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFAILSAFEBRAKINGDUTYCYCLE, NULL, NULL, 1,
	  "%d", failsafeBrakingEnabled));
}

API_PRETURN
PhidgetBLDCMotor_getFailsafeBrakingEnabled(PhidgetBLDCMotorHandle ch, int *failsafeBrakingEnabled) {

	TESTPTR_PR(ch);
	TESTPTR_PR(failsafeBrakingEnabled);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*failsafeBrakingEnabled = ch->failsafeBrakingEnabled;
	if (ch->failsafeBrakingEnabled == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setFailsafeCurrentLimit(PhidgetBLDCMotorHandle ch, double failsafeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFAILSAFECURRENTLIMIT, NULL, NULL, 1, "%g",
	  failsafeCurrentLimit));
}

API_PRETURN
PhidgetBLDCMotor_getFailsafeCurrentLimit(PhidgetBLDCMotorHandle ch, double *failsafeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(failsafeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*failsafeCurrentLimit = ch->failsafeCurrentLimit;
	if (ch->failsafeCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinFailsafeTime(PhidgetBLDCMotorHandle ch, uint32_t *minFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minFailsafeTime = ch->minFailsafeTime;
	if (ch->minFailsafeTime == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxFailsafeTime(PhidgetBLDCMotorHandle ch, uint32_t *maxFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxFailsafeTime = ch->maxFailsafeTime;
	if (ch->maxFailsafeTime == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setInductance(PhidgetBLDCMotorHandle ch, double inductance) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETINDUCTANCE, NULL, NULL, 1, "%g",
	  inductance));
}

API_PRETURN
PhidgetBLDCMotor_getInductance(PhidgetBLDCMotorHandle ch, double *inductance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(inductance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*inductance = ch->inductance;
	if (ch->inductance == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinInductance(PhidgetBLDCMotorHandle ch, double *minInductance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minInductance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minInductance = ch->minInductance;
	if (ch->minInductance == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxInductance(PhidgetBLDCMotorHandle ch, double *maxInductance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxInductance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxInductance = ch->maxInductance;
	if (ch->maxInductance == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getRescaleFactor(PhidgetBLDCMotorHandle ch, double *rescaleFactor) {

	TESTPTR_PR(ch);
	TESTPTR_PR(rescaleFactor);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*rescaleFactor = ch->rescaleFactor;
	if (ch->rescaleFactor == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setSurgeCurrentLimit(PhidgetBLDCMotorHandle ch, double surgeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSURGECURRENTLIMIT, NULL, NULL, 1, "%g",
	  surgeCurrentLimit));
}

API_PRETURN
PhidgetBLDCMotor_getSurgeCurrentLimit(PhidgetBLDCMotorHandle ch, double *surgeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(surgeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*surgeCurrentLimit = ch->surgeCurrentLimit;
	if (ch->surgeCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinSurgeCurrentLimit(PhidgetBLDCMotorHandle ch, double *minSurgeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minSurgeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minSurgeCurrentLimit = ch->minSurgeCurrentLimit;
	if (ch->minSurgeCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxSurgeCurrentLimit(PhidgetBLDCMotorHandle ch, double *maxSurgeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxSurgeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_BLDCMOTOR_100:
	case PHIDCHUID_DCC1100_BLDCMOTOR_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxSurgeCurrentLimit = ch->maxSurgeCurrentLimit;
	if (ch->maxSurgeCurrentLimit == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setTargetBrakingStrength(PhidgetBLDCMotorHandle ch, double targetBrakingStrength) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETBRAKINGDUTYCYCLE, NULL, NULL, 1, "%g",
	  targetBrakingStrength));
}

API_PRETURN
PhidgetBLDCMotor_getTargetBrakingStrength(PhidgetBLDCMotorHandle ch, double *targetBrakingStrength) {

	TESTPTR_PR(ch);
	TESTPTR_PR(targetBrakingStrength);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*targetBrakingStrength = ch->targetBrakingStrength;
	if (ch->targetBrakingStrength == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setTargetVelocity(PhidgetBLDCMotorHandle ch, double targetVelocity) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g",
	  targetVelocity));
}

API_VRETURN
PhidgetBLDCMotor_setTargetVelocity_async(PhidgetBLDCMotorHandle ch, double targetVelocity,
  Phidget_AsyncCallback fptr, void *ctx) {
	PhidgetReturnCode res;

	if (ch == NULL) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_INVALIDARG);
		return;
	}
	if (ch->phid.class != PHIDCHCLASS_BLDCMOTOR) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_WRONGDEVICE);
		return;
	}
	if (!ISATTACHED(ch)) {
		if (fptr) fptr((PhidgetHandle)ch, ctx, EPHIDGET_NOTATTACHED);
		return;
	}

	res = bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDUTYCYCLE, fptr, ctx, 1, "%g",
	  targetVelocity);
	if (res != EPHIDGET_OK && fptr != NULL)
		fptr((PhidgetHandle)ch, ctx, res);
}

API_PRETURN
PhidgetBLDCMotor_getTargetVelocity(PhidgetBLDCMotorHandle ch, double *targetVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(targetVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*targetVelocity = ch->targetVelocity;
	if (ch->targetVelocity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getVelocity(PhidgetBLDCMotorHandle ch, double *velocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(velocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*velocity = ch->velocity;
	if (ch->velocity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMinVelocity(PhidgetBLDCMotorHandle ch, double *minVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*minVelocity = ch->minVelocity;
	if (ch->minVelocity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_getMaxVelocity(PhidgetBLDCMotorHandle ch, double *maxVelocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxVelocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	*maxVelocity = ch->maxVelocity;
	if (ch->maxVelocity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setOnBrakingStrengthChangeHandler(PhidgetBLDCMotorHandle ch,
  PhidgetBLDCMotor_OnBrakingStrengthChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->BrakingStrengthChange = NULL;
		MEMORY_BARRIER();
		ch->BrakingStrengthChangeCtx = NULL;
	} else {
		ch->BrakingStrengthChangeCtx = ctx;
		MEMORY_BARRIER();
		ch->BrakingStrengthChange = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setOnPositionChangeHandler(PhidgetBLDCMotorHandle ch,
  PhidgetBLDCMotor_OnPositionChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->PositionChange = NULL;
		MEMORY_BARRIER();
		ch->PositionChangeCtx = NULL;
	} else {
		ch->PositionChangeCtx = ctx;
		MEMORY_BARRIER();
		ch->PositionChange = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetBLDCMotor_setOnVelocityUpdateHandler(PhidgetBLDCMotorHandle ch,
  PhidgetBLDCMotor_OnVelocityUpdateCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->VelocityUpdate = NULL;
		MEMORY_BARRIER();
		ch->VelocityUpdateCtx = NULL;
	} else {
		ch->VelocityUpdateCtx = ctx;
		MEMORY_BARRIER();
		ch->VelocityUpdate = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}
