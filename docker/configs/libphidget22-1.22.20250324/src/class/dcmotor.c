/* Generated: Wed Jan 27 2016 14:22:18 GMT-0700 (Mountain Standard Time) */
/* Will not be regenerated. */

#include "phidgetbase.h"
#include "class/dcmotor.gen.h"
#include "class/dcmotor.gen.c"

double PhidgetDCMotor_getLastBrakingStrength(PhidgetDCMotorHandle);

static void
PhidgetDCMotor_errorHandler(PhidgetChannelHandle phid, Phidget_ErrorEventCode code) {}

static void CCONV
PhidgetDCMotor_free(PhidgetChannelHandle *ch) {
	_free(ch);
}

API_PRETURN
PhidgetDCMotor_create(PhidgetDCMotorHandle *phidp) {
	return (_create(phidp));
}

static PhidgetReturnCode CCONV
PhidgetDCMotor_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	return (_setStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDCMotor_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	return (_getStatus(phid, bp));
}

static PhidgetReturnCode CCONV
PhidgetDCMotor_initAfterOpen(PhidgetChannelHandle phid) {
	return (_initAfterOpen(phid));
}

static PhidgetReturnCode CCONV
PhidgetDCMotor_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	ret = _setDefaults(phid);

	switch (phid->UCD->uid) {
#if (PHIDUID_DCC1010_SUPPORTED || PHIDUID_DCC1020_SUPPORTED || PHIDUID_DCC1030_SUPPORTED)
	#if PHIDUID_DCC1010_SUPPORTED
	case PHIDCHUID_DCC1010_DCMOTOR_100:
	#endif
	#if PHIDUID_DCC1020_SUPPORTED
	case PHIDCHUID_DCC1020_DCMOTOR_100:
	#endif
	#if PHIDUID_DCC1030_SUPPORTED
	case PHIDCHUID_DCC1030_DCMOTOR_100:
	#endif
		switch (ret) {
		case EPHIDGET_FAILSAFE:
			FIRE_ERROR(phid, EEPHIDGET_ESTOP, "External stop procedure initiated.");
			if (phid->iop != NULL) {
				mos_iop_addnotev(phid->iop, "ESTOP Button Pressed.");
			}
			break;
		case EPHIDGET_BADPOWER:
			FIRE_ERROR(phid, EEPHIDGET_BADPOWER, "Your power supply voltage is too high for the motor controller to begin operation.");
			if (phid->iop != NULL) {
				mos_iop_addnotev(phid->iop, "Bad Power Supply");
			}
			break;
		case EPHIDGET_POWERCYCLE:
			FIRE_ERROR(phid, EEPHIDGET_BADPOWER, "An overvoltage fault has triggered. Power your device off and on to resume operation. "
												 "We recommend a PowerGuard Phidget to prevent this in future.");
			if (phid->iop != NULL) {
				mos_iop_addnotev(phid->iop, "Overvoltage Fault. Power cycle required.");
			}
			break;
		case EPHIDGET_BADCURRENT:
			FIRE_ERROR(phid, EPHIDGET_BADCURRENT, "The current sensor is seeing an unacceptable offset in its readings. "
												  "Move the controller away from magnetic fields and try again.");
			if (phid->iop != NULL) {
				mos_iop_addnotev(phid->iop, "Current sensor error. Move away from magnets.");
			}
			break;
		default:
			break;
		}
		break;
#endif
	default:
		break;
	}

	return ret;
}

static PhidgetReturnCode
PhidgetDCMotor_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetDCMotorHandle ch;
	PhidgetReturnCode res;
	double tmpDbl;

	ch = (PhidgetDCMotorHandle)phid;

	switch (bp->vpkt) {
	case BP_SETDUTYCYCLE:
	case BP_SETBRAKINGDUTYCYCLE:
		TESTRANGE_IOP(bp->iop, "%lf", getBridgePacketDouble(bp, 0), -ch->maxVelocity, ch->maxVelocity);
		res = _bridgeInput(phid, bp);
		break;
	case BP_SETFAILSAFETIME:
		TESTRANGE_IOP(bp->iop, "%u", getBridgePacketUInt32(bp, 0), ch->minFailsafeTime, ch->maxFailsafeTime);
		res = _bridgeInput(phid, bp);
		break;
	case BP_INDUCTANCECHANGE:
		ch->inductance = getBridgePacketDouble(bp, 0);
		FIRE_PROPERTYCHANGE(ch, "Inductance");
		res = EPHIDGET_OK;
		break;
	case BP_DATAINTERVALCHANGE:
		if (bp->entrycnt > 1)
			ch->dataInterval = getBridgePacketDouble(bp, 1);
		else
			ch->dataInterval = (double)getBridgePacketUInt32(bp, 0);
		FIRE_PROPERTYCHANGE(ch, "DataInterval");
		FIRE_PROPERTYCHANGE(ch, "DataRate");
		res = EPHIDGET_OK;
		break;
	case BP_CURRENTLIMITCHANGE:
		ch->currentLimit = getBridgePacketDouble(bp, 0);
		FIRE_PROPERTYCHANGE(ch, "CurrentLimit");
		res = EPHIDGET_OK;
		break;
	case BP_SURGECURRENTLIMITCHANGE:
		ch->surgeCurrentLimit = getBridgePacketDouble(bp, 0);
		FIRE_PROPERTYCHANGE(ch, "SurgeCurrentLimit");
		res = EPHIDGET_OK;
		break;
	case BP_ACTIVECURRENTLIMITCHANGE:
		tmpDbl = getBridgePacketDouble(bp, 0);
		if (ch->activeCurrentLimit != tmpDbl) {
			ch->activeCurrentLimit = tmpDbl;
			FIRE_PROPERTYCHANGE(ch, "ActiveCurrentLimit");
		}
		res = EPHIDGET_OK;
		break;
	default:
		res = _bridgeInput(phid, bp);
		break;
	}

	return (res);
}

static void
PhidgetDCMotor_fireInitialEvents(PhidgetChannelHandle phid) {
	_fireInitialEvents(phid);
}

static int
PhidgetDCMotor_hasInitialState(PhidgetChannelHandle phid) {

	if (supportedBridgePacket(phid, BP_INDUCTANCECHANGE) && ((PhidgetDCMotorHandle)phid)->inductance == PUNK_DBL)
		return (PFALSE);

	return (_hasInitialState(phid));
}

double
PhidgetDCMotor_getLastBrakingStrength(PhidgetDCMotorHandle ch) {
	return (ch->brakingStrength);
}

API_PRETURN
PhidgetDCMotor_setBrakingEnabled(PhidgetDCMotorHandle ch, int brakingEnabled) {

	return PhidgetDCMotor_setTargetBrakingStrength(ch, brakingEnabled ? 1.0 : 0.0);
}

API_PRETURN
PhidgetDCMotor_getBrakingEnabled(PhidgetDCMotorHandle ch, int *brakingEnabled) {

	TESTPTR_PR(ch);
	TESTPTR_PR(brakingEnabled);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_BLDCMOTOR);
	TESTATTACHED_PR(ch);

	if (ch->targetBrakingStrength == (double)PUNK_DBL) {
		*brakingEnabled = PUNK_BOOL;
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	}

	*brakingEnabled = ch->targetBrakingStrength ? PTRUE : PFALSE;

	return (EPHIDGET_OK);
}
