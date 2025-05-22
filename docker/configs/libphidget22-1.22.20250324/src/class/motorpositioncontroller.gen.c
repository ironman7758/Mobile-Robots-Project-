/* Generated: Mon Mar 24 2025 13:18:57 GMT-0600 (Mountain Daylight Time) */

#include "device/vintdevice.h"
static void CCONV PhidgetMotorPositionController_errorHandler(PhidgetChannelHandle ch,
  Phidget_ErrorEventCode code);
static void CCONV PhidgetMotorPositionController_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetMotorPositionController_bridgeInput(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetMotorPositionController_setStatus(PhidgetChannelHandle phid,
  BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetMotorPositionController_getStatus(PhidgetChannelHandle phid,
  BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetMotorPositionController_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetMotorPositionController_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetMotorPositionController_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetMotorPositionController_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetMotorPositionController {
	struct _PhidgetChannel phid;
	int64_t positionOffset;
	double acceleration;
	double minAcceleration;
	double maxAcceleration;
	double activeCurrentLimit;
	double currentLimit;
	double minCurrentLimit;
	double maxCurrentLimit;
	double currentRegulatorGain;
	double minCurrentRegulatorGain;
	double maxCurrentRegulatorGain;
	double dataInterval;
	uint32_t minDataInterval;
	uint32_t maxDataInterval;
	double minDataRate;
	double maxDataRate;
	uint32_t deadBand;
	double dutyCycle;
	int engaged;
	int64_t expectedPosition;
	int enableExpectedPosition;
	int failsafeBrakingEnabled;
	double failsafeCurrentLimit;
	uint32_t minFailsafeTime;
	uint32_t maxFailsafeTime;
	Phidget_FanMode fanMode;
	double inductance;
	double minInductance;
	double maxInductance;
	Phidget_EncoderIOMode IOMode;
	double kd;
	double ki;
	double kp;
	int normalizePID;
	int64_t position;
	int64_t minPosition;
	int64_t maxPosition;
	Phidget_PositionType positionType;
	double rescaleFactor;
	double stallVelocity;
	double minStallVelocity;
	double maxStallVelocity;
	double surgeCurrentLimit;
	double minSurgeCurrentLimit;
	double maxSurgeCurrentLimit;
	int64_t targetPosition;
	double velocityLimit;
	double minVelocityLimit;
	double maxVelocityLimit;
	volatile PhidgetMotorPositionController_OnDutyCycleUpdateCallback DutyCycleUpdate;
	volatile void *DutyCycleUpdateCtx;
	volatile PhidgetMotorPositionController_OnExpectedPositionChangeCallback ExpectedPositionChange;
	volatile void *ExpectedPositionChangeCtx;
	volatile PhidgetMotorPositionController_OnPositionChangeCallback PositionChange;
	volatile void *PositionChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetMotorPositionControllerHandle ch;
	int nsversion;

	ch = (PhidgetMotorPositionControllerHandle)phid;

	nsversion = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (nsversion != 6) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 6 - functionality may be limited.", phid, nsversion);
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
	if (hasBridgePacketByName(bp, "currentLimit"))
		ch->currentLimit = getBridgePacketDoubleByName(bp, "currentLimit");
	if (hasBridgePacketByName(bp, "minCurrentLimit"))
		ch->minCurrentLimit = getBridgePacketDoubleByName(bp, "minCurrentLimit");
	if (hasBridgePacketByName(bp, "maxCurrentLimit"))
		ch->maxCurrentLimit = getBridgePacketDoubleByName(bp, "maxCurrentLimit");
	if (hasBridgePacketByName(bp, "currentRegulatorGain"))
		ch->currentRegulatorGain = getBridgePacketDoubleByName(bp, "currentRegulatorGain");
	if (hasBridgePacketByName(bp, "minCurrentRegulatorGain"))
		ch->minCurrentRegulatorGain = getBridgePacketDoubleByName(bp, "minCurrentRegulatorGain");
	if (hasBridgePacketByName(bp, "maxCurrentRegulatorGain"))
		ch->maxCurrentRegulatorGain = getBridgePacketDoubleByName(bp, "maxCurrentRegulatorGain");
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
	if (hasBridgePacketByName(bp, "deadBand"))
		ch->deadBand = getBridgePacketUInt32ByName(bp, "deadBand");
	if (hasBridgePacketByName(bp, "dutyCycle"))
		ch->dutyCycle = getBridgePacketDoubleByName(bp, "dutyCycle");
	if (hasBridgePacketByName(bp, "engaged"))
		ch->engaged = getBridgePacketInt32ByName(bp, "engaged");
	if (hasBridgePacketByName(bp, "expectedPosition"))
		ch->expectedPosition = getBridgePacketInt64ByName(bp, "expectedPosition");
	if (hasBridgePacketByName(bp, "enableExpectedPosition"))
		ch->enableExpectedPosition = getBridgePacketInt32ByName(bp, "enableExpectedPosition");
	if (hasBridgePacketByName(bp, "failsafeBrakingEnabled"))
		ch->failsafeBrakingEnabled = getBridgePacketInt32ByName(bp, "failsafeBrakingEnabled");
	if (hasBridgePacketByName(bp, "failsafeCurrentLimit"))
		ch->failsafeCurrentLimit = getBridgePacketDoubleByName(bp, "failsafeCurrentLimit");
	if (hasBridgePacketByName(bp, "minFailsafeTime"))
		ch->minFailsafeTime = getBridgePacketUInt32ByName(bp, "minFailsafeTime");
	if (hasBridgePacketByName(bp, "maxFailsafeTime"))
		ch->maxFailsafeTime = getBridgePacketUInt32ByName(bp, "maxFailsafeTime");
	if (hasBridgePacketByName(bp, "fanMode"))
		ch->fanMode = getBridgePacketInt32ByName(bp, "fanMode");
	if (hasBridgePacketByName(bp, "inductance"))
		ch->inductance = getBridgePacketDoubleByName(bp, "inductance");
	if (hasBridgePacketByName(bp, "minInductance"))
		ch->minInductance = getBridgePacketDoubleByName(bp, "minInductance");
	if (hasBridgePacketByName(bp, "maxInductance"))
		ch->maxInductance = getBridgePacketDoubleByName(bp, "maxInductance");
	if (hasBridgePacketByName(bp, "IOMode"))
		ch->IOMode = getBridgePacketInt32ByName(bp, "IOMode");
	if (hasBridgePacketByName(bp, "kd"))
		ch->kd = getBridgePacketDoubleByName(bp, "kd");
	if (hasBridgePacketByName(bp, "ki"))
		ch->ki = getBridgePacketDoubleByName(bp, "ki");
	if (hasBridgePacketByName(bp, "kp"))
		ch->kp = getBridgePacketDoubleByName(bp, "kp");
	if (hasBridgePacketByName(bp, "normalizePID"))
		ch->normalizePID = getBridgePacketInt32ByName(bp, "normalizePID");
	if (hasBridgePacketByName(bp, "position"))
		ch->position = getBridgePacketInt64ByName(bp, "position");
	if (hasBridgePacketByName(bp, "minPosition"))
		ch->minPosition = getBridgePacketInt64ByName(bp, "minPosition");
	if (hasBridgePacketByName(bp, "maxPosition"))
		ch->maxPosition = getBridgePacketInt64ByName(bp, "maxPosition");
	if (hasBridgePacketByName(bp, "positionType"))
		ch->positionType = getBridgePacketInt32ByName(bp, "positionType");
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
	if (hasBridgePacketByName(bp, "targetPosition"))
		ch->targetPosition = getBridgePacketInt64ByName(bp, "targetPosition");
	if (hasBridgePacketByName(bp, "velocityLimit"))
		ch->velocityLimit = getBridgePacketDoubleByName(bp, "velocityLimit");
	if (hasBridgePacketByName(bp, "minVelocityLimit"))
		ch->minVelocityLimit = getBridgePacketDoubleByName(bp, "minVelocityLimit");
	if (hasBridgePacketByName(bp, "maxVelocityLimit"))
		ch->maxVelocityLimit = getBridgePacketDoubleByName(bp, "maxVelocityLimit");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetMotorPositionControllerHandle ch;

	ch = (PhidgetMotorPositionControllerHandle)phid;

	return (createBridgePacket(bp, BP_SETSTATUS, 51, "_class_version_=%u"
	  ",positionOffset=%l"
	  ",acceleration=%g"
	  ",minAcceleration=%g"
	  ",maxAcceleration=%g"
	  ",activeCurrentLimit=%g"
	  ",currentLimit=%g"
	  ",minCurrentLimit=%g"
	  ",maxCurrentLimit=%g"
	  ",currentRegulatorGain=%g"
	  ",minCurrentRegulatorGain=%g"
	  ",maxCurrentRegulatorGain=%g"
	  ",dataInterval=%u"
	  ",minDataInterval=%u"
	  ",maxDataInterval=%u"
	  ",dataIntervalDbl=%g"
	  ",minDataRate=%g"
	  ",maxDataRate=%g"
	  ",deadBand=%u"
	  ",dutyCycle=%g"
	  ",engaged=%d"
	  ",expectedPosition=%l"
	  ",enableExpectedPosition=%d"
	  ",failsafeBrakingEnabled=%d"
	  ",failsafeCurrentLimit=%g"
	  ",minFailsafeTime=%u"
	  ",maxFailsafeTime=%u"
	  ",fanMode=%d"
	  ",inductance=%g"
	  ",minInductance=%g"
	  ",maxInductance=%g"
	  ",IOMode=%d"
	  ",kd=%g"
	  ",ki=%g"
	  ",kp=%g"
	  ",normalizePID=%d"
	  ",position=%l"
	  ",minPosition=%l"
	  ",maxPosition=%l"
	  ",positionType=%d"
	  ",rescaleFactor=%g"
	  ",stallVelocity=%g"
	  ",minStallVelocity=%g"
	  ",maxStallVelocity=%g"
	  ",surgeCurrentLimit=%g"
	  ",minSurgeCurrentLimit=%g"
	  ",maxSurgeCurrentLimit=%g"
	  ",targetPosition=%l"
	  ",velocityLimit=%g"
	  ",minVelocityLimit=%g"
	  ",maxVelocityLimit=%g"
	  ,6 /* class version */
	  ,ch->positionOffset
	  ,ch->acceleration
	  ,ch->minAcceleration
	  ,ch->maxAcceleration
	  ,ch->activeCurrentLimit
	  ,ch->currentLimit
	  ,ch->minCurrentLimit
	  ,ch->maxCurrentLimit
	  ,ch->currentRegulatorGain
	  ,ch->minCurrentRegulatorGain
	  ,ch->maxCurrentRegulatorGain
	  ,(uint32_t)round(ch->dataInterval)
	  ,ch->minDataInterval
	  ,ch->maxDataInterval
	  ,ch->dataInterval
	  ,ch->minDataRate
	  ,ch->maxDataRate
	  ,ch->deadBand
	  ,ch->dutyCycle
	  ,ch->engaged
	  ,ch->expectedPosition
	  ,ch->enableExpectedPosition
	  ,ch->failsafeBrakingEnabled
	  ,ch->failsafeCurrentLimit
	  ,ch->minFailsafeTime
	  ,ch->maxFailsafeTime
	  ,ch->fanMode
	  ,ch->inductance
	  ,ch->minInductance
	  ,ch->maxInductance
	  ,ch->IOMode
	  ,ch->kd
	  ,ch->ki
	  ,ch->kp
	  ,ch->normalizePID
	  ,ch->position
	  ,ch->minPosition
	  ,ch->maxPosition
	  ,ch->positionType
	  ,ch->rescaleFactor
	  ,ch->stallVelocity
	  ,ch->minStallVelocity
	  ,ch->maxStallVelocity
	  ,ch->surgeCurrentLimit
	  ,ch->minSurgeCurrentLimit
	  ,ch->maxSurgeCurrentLimit
	  ,ch->targetPosition
	  ,ch->velocityLimit
	  ,ch->minVelocityLimit
	  ,ch->maxVelocityLimit
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetMotorPositionControllerHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetMotorPositionControllerHandle)phid;
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
	case BP_SETCURRENTREGULATORGAIN:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minCurrentRegulatorGain,
		  ch->maxCurrentRegulatorGain);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->currentRegulatorGain = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "CurrentRegulatorGain");
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
	case BP_SETDEADBAND:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->deadBand = getBridgePacketUInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "DeadBand");
		}
		break;
	case BP_SETENGAGED:
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->engaged = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Engaged");
		}
		break;
	case BP_SETENABLEEXPECTEDPOSITION:
		TESTBOOL_IOP(bp->iop, getBridgePacketInt32(bp, 0));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->enableExpectedPosition = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "EnableExpectedPosition");
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
	case BP_SETFANMODE:
		if (!supportedFanMode(phid, (Phidget_FanMode)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified FanMode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->fanMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "FanMode");
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
	case BP_SETIOMODE:
		if (!supportedEncoderIOMode(phid, (Phidget_EncoderIOMode)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified EncoderIOMode is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->IOMode = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "IOMode");
		}
		break;
	case BP_SETKD:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->kd = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Kd");
		}
		break;
	case BP_SETKI:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->ki = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Ki");
		}
		break;
	case BP_SETKP:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->kp = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "Kp");
		}
		break;
	case BP_POSITIONTYPE:
		if (!supportedPositionType(phid, (Phidget_PositionType)getBridgePacketInt32(bp, 0)))
			return (MOS_ERROR(bp->iop, EPHIDGET_INVALIDARG,
			  "Specified PositionType is unsupported by this device."));
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->positionType = getBridgePacketInt32(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "PositionType");
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
	case BP_SETTARGETPOSITION:
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->targetPosition = getBridgePacketInt64(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "TargetPosition");
		}
		break;
	case BP_SETDUTYCYCLE:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), ch->minVelocityLimit,
		  ch->maxVelocityLimit);
		res = DEVBRIDGEINPUT(phid, bp);
		if (res != EPHIDGET_OK)
			break;
		ch->velocityLimit = getBridgePacketDouble(bp, 0);
		if (bridgePacketIsFromNet(bp)) {
			FIRE_PROPERTYCHANGE(ch, "VelocityLimit");
		}
		break;
	case BP_DUTYCYCLECHANGE:
		ch->dutyCycle = getBridgePacketDouble(bp, 0);
		FIRECH(ch, PhidgetMotorPositionController, DutyCycleUpdate, ch->dutyCycle);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetMotorPositionControllerHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetMotorPositionControllerHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		ch->dataInterval = 100;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->currentLimit = 2;
		ch->maxCurrentLimit = 25;
		ch->minCurrentLimit = 2;
		ch->currentRegulatorGain = 10;
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->maxCurrentRegulatorGain = 100;
		ch->minCurrentRegulatorGain = 1;
		ch->velocityLimit = 20000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 250000;
		ch->maxAcceleration = 10000000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 10000;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->rescaleFactor = 1;
		ch->fanMode = FAN_MODE_AUTO;
		ch->deadBand = 0;
		ch->kp = 2000;
		ch->kd = 25000;
		ch->ki = 5;
		ch->IOMode = ENCODER_IO_MODE_PUSH_PULL;
		ch->normalizePID = 0;
		break;
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		ch->dataInterval = 100;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->currentLimit = 2;
		ch->maxCurrentLimit = 25;
		ch->minCurrentLimit = 2;
		ch->currentRegulatorGain = 10;
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->maxCurrentRegulatorGain = 100;
		ch->minCurrentRegulatorGain = 1;
		ch->velocityLimit = 20000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 250000;
		ch->maxAcceleration = 10000000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 10000;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->rescaleFactor = 1;
		ch->fanMode = FAN_MODE_AUTO;
		ch->deadBand = 0;
		ch->kp = 2000;
		ch->kd = 25000;
		ch->ki = 5;
		ch->IOMode = ENCODER_IO_MODE_PUSH_PULL;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		ch->normalizePID = 0;
		break;
	case PHIDCHUID_DCC1020_MOTORPOSITIONCONTROLLER_100:
		ch->dataInterval = 100;
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
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->velocityLimit = 20000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 500000;
		ch->maxAcceleration = 10000000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 10000;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->enableExpectedPosition = 0;
		ch->expectedPosition = PUNK_INT64;
		ch->rescaleFactor = 1;
		ch->deadBand = 0;
		ch->kp = 0.001;
		ch->kd = 0.00001;
		ch->ki = 0.0001;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		ch->inductance = PUNK_DBL;
		ch->maxInductance = 0.01;
		ch->minInductance = 0.0001;
		ch->failsafeCurrentLimit = 5;
		ch->failsafeBrakingEnabled = 0;
		break;
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
		ch->dataInterval = 100;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->currentLimit = 1;
		ch->maxCurrentLimit = 2;
		ch->minCurrentLimit = 0;
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->velocityLimit = 20000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 250000;
		ch->maxAcceleration = 10000000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 10000;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->rescaleFactor = 1;
		ch->deadBand = 0;
		ch->kp = 2000;
		ch->kd = 25000;
		ch->ki = 5;
		ch->normalizePID = 0;
		break;
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
		ch->dataInterval = 100;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->currentLimit = 1;
		ch->maxCurrentLimit = 2;
		ch->minCurrentLimit = 0;
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->velocityLimit = 20000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 250000;
		ch->maxAcceleration = 10000000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 10000;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->rescaleFactor = 1;
		ch->deadBand = 0;
		ch->kp = 2000;
		ch->kd = 25000;
		ch->ki = 5;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		ch->normalizePID = 0;
		break;
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
		ch->dataInterval = 100;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->currentLimit = 1;
		ch->maxCurrentLimit = 3.5;
		ch->minCurrentLimit = 0;
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->velocityLimit = 20000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 250000;
		ch->maxAcceleration = 10000000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 10000;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->rescaleFactor = 1;
		ch->deadBand = 0;
		ch->kp = 2000;
		ch->kd = 25000;
		ch->ki = 5;
		ch->normalizePID = 0;
		break;
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
		ch->dataInterval = 100;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->currentLimit = 1;
		ch->maxCurrentLimit = 4;
		ch->minCurrentLimit = 0;
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->velocityLimit = 20000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 250000;
		ch->maxAcceleration = 10000000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 10000;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->rescaleFactor = 1;
		ch->deadBand = 0;
		ch->kp = 2000;
		ch->kd = 25000;
		ch->ki = 5;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		ch->normalizePID = 0;
		break;
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
		ch->dataInterval = 100;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->velocityLimit = 1000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 10000;
		ch->maxAcceleration = 100000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 100;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->rescaleFactor = 1;
		ch->deadBand = 0;
		ch->kp = 20000;
		ch->kd = 40000;
		ch->ki = 2;
		ch->stallVelocity = 400;
		ch->minStallVelocity = 0;
		ch->maxStallVelocity = 2000;
		ch->normalizePID = 0;
		break;
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		ch->dataInterval = 100;
		ch->minDataInterval = 20;
		ch->maxDataInterval = 60000;
		ch->minDataRate = 0.016666666666666666;
		ch->maxDataRate = 50;
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->velocityLimit = 1000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 10000;
		ch->maxAcceleration = 100000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 100;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->rescaleFactor = 1;
		ch->deadBand = 0;
		ch->kp = 20000;
		ch->kd = 40000;
		ch->ki = 2;
		ch->stallVelocity = 400;
		ch->minStallVelocity = 0;
		ch->maxStallVelocity = 2000;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		ch->normalizePID = 0;
		break;
	case PHIDCHUID_DCC1120_MOTORPOSITIONCONTROLLER_100:
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
		ch->dutyCycle = PUNK_DBL;
		ch->engaged = 0;
		ch->velocityLimit = 2000;
		ch->minVelocityLimit = 0;
		ch->maxVelocityLimit = 2000;
		ch->maxAcceleration = 10000000;
		ch->minAcceleration = 0.1;
		ch->acceleration = 10000;
		ch->targetPosition = 0;
		ch->position = PUNK_INT64;
		ch->maxPosition = 1000000000000000;
		ch->minPosition = -1000000000000000;
		ch->enableExpectedPosition = 0;
		ch->expectedPosition = PUNK_INT64;
		ch->rescaleFactor = 1;
		ch->deadBand = 0;
		ch->kp = 0.001;
		ch->kd = 0.00001;
		ch->ki = 0.0001;
		ch->maxFailsafeTime = 30000;
		ch->minFailsafeTime = 500;
		ch->inductance = PUNK_DBL;
		ch->maxInductance = 0.01;
		ch->minInductance = 0.0001;
		ch->positionType = POSITION_TYPE_HALL_SENSOR;
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
	PhidgetMotorPositionControllerHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ch = (PhidgetMotorPositionControllerHandle)phid;
	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
		break;
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
		break;
	case PHIDCHUID_DCC1020_MOTORPOSITIONCONTROLLER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, 1, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETENABLEEXPECTEDPOSITION, NULL, NULL, 1, "%d",
		  ch->enableExpectedPosition);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, 1, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETENGAGED, NULL, NULL, 1, "%d", ch->engaged);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, 1, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETTARGETPOSITION, NULL, NULL, 1, "%l", ch->targetPosition);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDEADBAND, NULL, NULL, 1, "%u", ch->deadBand);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKP, NULL, NULL, 1, "%g", ch->kp);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKD, NULL, NULL, 1, "%g", ch->kd);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKI, NULL, NULL, 1, "%g", ch->ki);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, 1, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETENGAGED, NULL, NULL, 1, "%d", ch->engaged);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, 1, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETTARGETPOSITION, NULL, NULL, 1, "%l", ch->targetPosition);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDEADBAND, NULL, NULL, 1, "%u", ch->deadBand);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKP, NULL, NULL, 1, "%g", ch->kp);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKD, NULL, NULL, 1, "%g", ch->kd);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKI, NULL, NULL, 1, "%g", ch->ki);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, 1, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETENGAGED, NULL, NULL, 1, "%d", ch->engaged);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, 1, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETTARGETPOSITION, NULL, NULL, 1, "%l", ch->targetPosition);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDEADBAND, NULL, NULL, 1, "%u", ch->deadBand);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKP, NULL, NULL, 1, "%g", ch->kp);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKD, NULL, NULL, 1, "%g", ch->kd);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKI, NULL, NULL, 1, "%g", ch->ki);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, 1, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETENGAGED, NULL, NULL, 1, "%d", ch->engaged);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, 1, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETTARGETPOSITION, NULL, NULL, 1, "%l", ch->targetPosition);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDEADBAND, NULL, NULL, 1, "%u", ch->deadBand);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKP, NULL, NULL, 1, "%g", ch->kp);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKD, NULL, NULL, 1, "%g", ch->kd);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKI, NULL, NULL, 1, "%g", ch->ki);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETENGAGED, NULL, NULL, 1, "%d", ch->engaged);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, 1, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETTARGETPOSITION, NULL, NULL, 1, "%l", ch->targetPosition);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDEADBAND, NULL, NULL, 1, "%u", ch->deadBand);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKP, NULL, NULL, 1, "%g", ch->kp);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKD, NULL, NULL, 1, "%g", ch->kd);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKI, NULL, NULL, 1, "%g", ch->ki);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSTALLVELOCITY, NULL, NULL, 1, "%g", ch->stallVelocity);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		ret = bridgeSendToDevice(phid, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
		  (uint32_t)round(ch->dataInterval));
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETENGAGED, NULL, NULL, 1, "%d", ch->engaged);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDUTYCYCLE, NULL, NULL, 1, "%g", ch->velocityLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETACCELERATION, NULL, NULL, 1, "%g", ch->acceleration);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETTARGETPOSITION, NULL, NULL, 1, "%l", ch->targetPosition);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETDEADBAND, NULL, NULL, 1, "%u", ch->deadBand);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKP, NULL, NULL, 1, "%g", ch->kp);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKD, NULL, NULL, 1, "%g", ch->kd);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETKI, NULL, NULL, 1, "%g", ch->ki);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETSTALLVELOCITY, NULL, NULL, 1, "%g", ch->stallVelocity);
		if (ret != EPHIDGET_OK)
			break;
		break;
	case PHIDCHUID_DCC1120_MOTORPOSITIONCONTROLLER_100:
		ret = bridgeSendToDevice(phid, BP_SETCURRENTLIMIT, NULL, NULL, 1, "%g", ch->currentLimit);
		if (ret != EPHIDGET_OK)
			break;
		ret = bridgeSendToDevice(phid, BP_SETENABLEEXPECTEDPOSITION, NULL, NULL, 1, "%d",
		  ch->enableExpectedPosition);
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
	PhidgetMotorPositionControllerHandle ch;

	ch = (PhidgetMotorPositionControllerHandle)phid;

	if(ch->dutyCycle != PUNK_DBL)
		FIRECH(ch, PhidgetMotorPositionController, DutyCycleUpdate, ch->dutyCycle);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetMotorPositionControllerHandle ch;

	ch = (PhidgetMotorPositionControllerHandle)phid;

	if(ch->dutyCycle == PUNK_DBL)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetMotorPositionController));
}

static PhidgetReturnCode CCONV
_create(PhidgetMotorPositionControllerHandle *phidp) {

	CHANNELCREATE_BODY(MotorPositionController, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_delete(PhidgetMotorPositionControllerHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetMotorPositionController_enableFailsafe(PhidgetMotorPositionControllerHandle ch,
  uint32_t failsafeTime) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFAILSAFETIME, NULL, NULL, 1, "%u",
	  failsafeTime); 
}

API_PRETURN
PhidgetMotorPositionController_resetFailsafe(PhidgetMotorPositionControllerHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_FAILSAFERESET, NULL, NULL, 0, NULL); 
}

API_PRETURN
PhidgetMotorPositionController_getActiveCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *activeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(activeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_setCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double currentLimit) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCURRENTLIMIT, NULL, NULL, 1, "%g",
	  currentLimit));
}

API_PRETURN
PhidgetMotorPositionController_getCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *currentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(currentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_getMinCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *minCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_getMaxCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *maxCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_setCurrentRegulatorGain(PhidgetMotorPositionControllerHandle ch,
  double currentRegulatorGain) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETCURRENTREGULATORGAIN, NULL, NULL, 1, "%g",
	  currentRegulatorGain));
}

API_PRETURN
PhidgetMotorPositionController_getCurrentRegulatorGain(PhidgetMotorPositionControllerHandle ch,
  double *currentRegulatorGain) {

	TESTPTR_PR(ch);
	TESTPTR_PR(currentRegulatorGain);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1020_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1120_MOTORPOSITIONCONTROLLER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*currentRegulatorGain = ch->currentRegulatorGain;
	if (ch->currentRegulatorGain == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMinCurrentRegulatorGain(PhidgetMotorPositionControllerHandle ch,
  double *minCurrentRegulatorGain) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minCurrentRegulatorGain);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1020_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1120_MOTORPOSITIONCONTROLLER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*minCurrentRegulatorGain = ch->minCurrentRegulatorGain;
	if (ch->minCurrentRegulatorGain == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMaxCurrentRegulatorGain(PhidgetMotorPositionControllerHandle ch,
  double *maxCurrentRegulatorGain) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxCurrentRegulatorGain);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1020_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1120_MOTORPOSITIONCONTROLLER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*maxCurrentRegulatorGain = ch->maxCurrentRegulatorGain;
	if (ch->maxCurrentRegulatorGain == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setDataInterval(PhidgetMotorPositionControllerHandle ch,
  uint32_t dataInterval) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, 1, "%u",
	  dataInterval));
}

API_PRETURN
PhidgetMotorPositionController_getDataInterval(PhidgetMotorPositionControllerHandle ch,
  uint32_t *dataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	*dataInterval = (uint32_t)round(ch->dataInterval);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMinDataInterval(PhidgetMotorPositionControllerHandle ch,
  uint32_t *minDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	*minDataInterval = ch->minDataInterval;
	if (ch->minDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMaxDataInterval(PhidgetMotorPositionControllerHandle ch,
  uint32_t *maxDataInterval) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataInterval);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	*maxDataInterval = ch->maxDataInterval;
	if (ch->maxDataInterval == (uint32_t)PUNK_UINT32)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setDataRate(PhidgetMotorPositionControllerHandle ch, double dataRate) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETDATAINTERVAL, NULL, NULL, 2, "%u%g",
	  (uint32_t)round(1000.0 / dataRate), (double)(1000.0 / dataRate));
}

API_PRETURN
PhidgetMotorPositionController_getDataRate(PhidgetMotorPositionControllerHandle ch, double *dataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	*dataRate = round_double((1000.0 / ch->dataInterval), 4);
	if (ch->dataInterval == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMinDataRate(PhidgetMotorPositionControllerHandle ch,
  double *minDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	*minDataRate = ch->minDataRate;
	if (ch->minDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getMaxDataRate(PhidgetMotorPositionControllerHandle ch,
  double *maxDataRate) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxDataRate);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	*maxDataRate = ch->maxDataRate;
	if (ch->maxDataRate == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getDutyCycle(PhidgetMotorPositionControllerHandle ch,
  double *dutyCycle) {

	TESTPTR_PR(ch);
	TESTPTR_PR(dutyCycle);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	*dutyCycle = ch->dutyCycle;
	if (ch->dutyCycle == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setEngaged(PhidgetMotorPositionControllerHandle ch, int engaged) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENGAGED, NULL, NULL, 1, "%d", engaged));
}

API_PRETURN
PhidgetMotorPositionController_getEngaged(PhidgetMotorPositionControllerHandle ch, int *engaged) {

	TESTPTR_PR(ch);
	TESTPTR_PR(engaged);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	*engaged = ch->engaged;
	if (ch->engaged == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setEnableExpectedPosition(PhidgetMotorPositionControllerHandle ch,
  int enableExpectedPosition) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETENABLEEXPECTEDPOSITION, NULL, NULL, 1,
	  "%d", enableExpectedPosition));
}

API_PRETURN
PhidgetMotorPositionController_getEnableExpectedPosition(PhidgetMotorPositionControllerHandle ch,
  int *enableExpectedPosition) {

	TESTPTR_PR(ch);
	TESTPTR_PR(enableExpectedPosition);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*enableExpectedPosition = ch->enableExpectedPosition;
	if (ch->enableExpectedPosition == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setFailsafeBrakingEnabled(PhidgetMotorPositionControllerHandle ch,
  int failsafeBrakingEnabled) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFAILSAFEBRAKINGDUTYCYCLE, NULL, NULL, 1,
	  "%d", failsafeBrakingEnabled));
}

API_PRETURN
PhidgetMotorPositionController_getFailsafeBrakingEnabled(PhidgetMotorPositionControllerHandle ch,
  int *failsafeBrakingEnabled) {

	TESTPTR_PR(ch);
	TESTPTR_PR(failsafeBrakingEnabled);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_setFailsafeCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double failsafeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFAILSAFECURRENTLIMIT, NULL, NULL, 1, "%g",
	  failsafeCurrentLimit));
}

API_PRETURN
PhidgetMotorPositionController_getFailsafeCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *failsafeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(failsafeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_getMinFailsafeTime(PhidgetMotorPositionControllerHandle ch,
  uint32_t *minFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
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
PhidgetMotorPositionController_getMaxFailsafeTime(PhidgetMotorPositionControllerHandle ch,
  uint32_t *maxFailsafeTime) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxFailsafeTime);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
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
PhidgetMotorPositionController_setFanMode(PhidgetMotorPositionControllerHandle ch,
  Phidget_FanMode fanMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETFANMODE, NULL, NULL, 1, "%d", fanMode));
}

API_PRETURN
PhidgetMotorPositionController_getFanMode(PhidgetMotorPositionControllerHandle ch,
  Phidget_FanMode *fanMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(fanMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1020_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1120_MOTORPOSITIONCONTROLLER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*fanMode = ch->fanMode;
	if (ch->fanMode == (Phidget_FanMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setInductance(PhidgetMotorPositionControllerHandle ch,
  double inductance) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETINDUCTANCE, NULL, NULL, 1, "%g",
	  inductance));
}

API_PRETURN
PhidgetMotorPositionController_getInductance(PhidgetMotorPositionControllerHandle ch,
  double *inductance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(inductance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_getMinInductance(PhidgetMotorPositionControllerHandle ch,
  double *minInductance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minInductance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_getMaxInductance(PhidgetMotorPositionControllerHandle ch,
  double *maxInductance) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxInductance);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_setIOMode(PhidgetMotorPositionControllerHandle ch,
  Phidget_EncoderIOMode IOMode) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETIOMODE, NULL, NULL, 1, "%d", IOMode));
}

API_PRETURN
PhidgetMotorPositionController_getIOMode(PhidgetMotorPositionControllerHandle ch,
  Phidget_EncoderIOMode *IOMode) {

	TESTPTR_PR(ch);
	TESTPTR_PR(IOMode);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1020_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1120_MOTORPOSITIONCONTROLLER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*IOMode = ch->IOMode;
	if (ch->IOMode == (Phidget_EncoderIOMode)PUNK_ENUM)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getNormalizePID(PhidgetMotorPositionControllerHandle ch,
  int *normalizePID) {

	TESTPTR_PR(ch);
	TESTPTR_PR(normalizePID);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1020_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1120_MOTORPOSITIONCONTROLLER_100:
		return (PHID_RETURN(EPHIDGET_UNSUPPORTED));
	default:
		break;
	}

	*normalizePID = ch->normalizePID;
	if (ch->normalizePID == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_getRescaleFactor(PhidgetMotorPositionControllerHandle ch,
  double *rescaleFactor) {

	TESTPTR_PR(ch);
	TESTPTR_PR(rescaleFactor);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	*rescaleFactor = ch->rescaleFactor;
	if (ch->rescaleFactor == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setSurgeCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double surgeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	return (bridgeSendToDevice((PhidgetChannelHandle)ch, BP_SETSURGECURRENTLIMIT, NULL, NULL, 1, "%g",
	  surgeCurrentLimit));
}

API_PRETURN
PhidgetMotorPositionController_getSurgeCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *surgeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(surgeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_getMinSurgeCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *minSurgeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(minSurgeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_getMaxSurgeCurrentLimit(PhidgetMotorPositionControllerHandle ch,
  double *maxSurgeCurrentLimit) {

	TESTPTR_PR(ch);
	TESTPTR_PR(maxSurgeCurrentLimit);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);
	TESTATTACHED_PR(ch);

	switch (ch->phid.UCD->uid) {
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_200:
	case PHIDCHUID_DCC1000_MOTORPOSITIONCONTROLLER_210:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1001_MOTORPOSITIONCONTROLLER_120:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1002_MOTORPOSITIONCONTROLLER_110:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_100:
	case PHIDCHUID_DCC1100_MOTORPOSITIONCONTROLLER_120:
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
PhidgetMotorPositionController_setOnDutyCycleUpdateHandler(PhidgetMotorPositionControllerHandle ch,
  PhidgetMotorPositionController_OnDutyCycleUpdateCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->DutyCycleUpdate = NULL;
		MEMORY_BARRIER();
		ch->DutyCycleUpdateCtx = NULL;
	} else {
		ch->DutyCycleUpdateCtx = ctx;
		MEMORY_BARRIER();
		ch->DutyCycleUpdate = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setOnExpectedPositionChangeHandler(PhidgetMotorPositionControllerHandle ch,
  PhidgetMotorPositionController_OnExpectedPositionChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->ExpectedPositionChange = NULL;
		MEMORY_BARRIER();
		ch->ExpectedPositionChangeCtx = NULL;
	} else {
		ch->ExpectedPositionChangeCtx = ctx;
		MEMORY_BARRIER();
		ch->ExpectedPositionChange = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetMotorPositionController_setOnPositionChangeHandler(PhidgetMotorPositionControllerHandle ch,
  PhidgetMotorPositionController_OnPositionChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_MOTORPOSITIONCONTROLLER);

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
