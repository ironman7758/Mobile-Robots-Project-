/* Generated: Mon Mar 24 2025 13:18:57 GMT-0600 (Mountain Daylight Time) */

#include "device/gpsdevice.h"
static void CCONV PhidgetGPS_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetGPS_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetGPS_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetGPS_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetGPS_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetGPS_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetGPS_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetGPS_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetGPS_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetGPS {
	struct _PhidgetChannel phid;
	double altitude;
	PhidgetGPS_Date date;
	uint8_t dateValid;
	double heading;
	double latitude;
	double longitude;
	PhidgetGPS_NMEAData NMEAData;
	uint8_t NMEADataValid;
	int positionFixState;
	PhidgetGPS_Time time;
	uint8_t timeValid;
	double velocity;
	volatile PhidgetGPS_OnHeadingChangeCallback HeadingChange;
	volatile void *HeadingChangeCtx;
	volatile PhidgetGPS_OnPositionChangeCallback PositionChange;
	volatile void *PositionChangeCtx;
	volatile PhidgetGPS_OnPositionFixStateChangeCallback PositionFixStateChange;
	volatile void *PositionFixStateChangeCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetGPSHandle ch;
	int nsversion;

	ch = (PhidgetGPSHandle)phid;

	nsversion = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (nsversion != 0) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 0 - functionality may be limited.", phid, nsversion);
	}

	if (hasBridgePacketByName(bp, "altitude"))
		ch->altitude = getBridgePacketDoubleByName(bp, "altitude");
	if (hasBridgePacketByName(bp, "heading"))
		ch->heading = getBridgePacketDoubleByName(bp, "heading");
	if (hasBridgePacketByName(bp, "latitude"))
		ch->latitude = getBridgePacketDoubleByName(bp, "latitude");
	if (hasBridgePacketByName(bp, "longitude"))
		ch->longitude = getBridgePacketDoubleByName(bp, "longitude");
	if (hasBridgePacketByName(bp, "positionFixState"))
		ch->positionFixState = getBridgePacketInt32ByName(bp, "positionFixState");
	if (hasBridgePacketByName(bp, "velocity"))
		ch->velocity = getBridgePacketDoubleByName(bp, "velocity");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetGPSHandle ch;

	ch = (PhidgetGPSHandle)phid;

	return (createBridgePacket(bp, BP_SETSTATUS, 7, "_class_version_=%u"
	  ",altitude=%g"
	  ",heading=%g"
	  ",latitude=%g"
	  ",longitude=%g"
	  ",positionFixState=%d"
	  ",velocity=%g"
	  ,0 /* class version */
	  ,ch->altitude
	  ,ch->heading
	  ,ch->latitude
	  ,ch->longitude
	  ,ch->positionFixState
	  ,ch->velocity
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetGPSHandle ch;
	PhidgetReturnCode res;

	ch = (PhidgetGPSHandle)phid;
	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_HEADINGCHANGE:
		ch->heading = getBridgePacketDouble(bp, 0);
		ch->velocity = getBridgePacketDouble(bp, 1);
		FIRECH(ch, PhidgetGPS, HeadingChange, ch->heading, ch->velocity);
		break;
	case BP_POSITIONCHANGE:
		ch->latitude = getBridgePacketDouble(bp, 0);
		ch->longitude = getBridgePacketDouble(bp, 1);
		ch->altitude = getBridgePacketDouble(bp, 2);
		FIRECH(ch, PhidgetGPS, PositionChange, ch->latitude, ch->longitude, ch->altitude);
		break;
	case BP_POSITIONFIXSTATUSCHANGE:
		ch->positionFixState = getBridgePacketInt32(bp, 0);
		FIRECH(ch, PhidgetGPS, PositionFixStateChange, ch->positionFixState);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetGPSDeviceHandle parentGPS;
	PhidgetGPSHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetGPSHandle)phid;

	ret = EPHIDGET_OK;

	parentGPS = (PhidgetGPSDeviceHandle)phid->parent;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1040_GPS_000:
		ch->altitude = parentGPS->altitude[ch->phid.index];
		ch->dateValid = parentGPS->dateValid[ch->phid.index];
		ch->date = parentGPS->date[ch->phid.index];
		ch->heading = parentGPS->heading[ch->phid.index];
		ch->latitude = parentGPS->latitude[ch->phid.index];
		ch->longitude = parentGPS->longitude[ch->phid.index];
		ch->NMEADataValid = parentGPS->NMEADataValid[ch->phid.index];
		ch->NMEAData = parentGPS->NMEAData[ch->phid.index];
		ch->positionFixState = parentGPS->positionFixState[ch->phid.index];
		ch->timeValid = parentGPS->timeValid[ch->phid.index];
		ch->time = parentGPS->time[ch->phid.index];
		ch->velocity = parentGPS->velocity[ch->phid.index];
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}


	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1040_GPS_000:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {
	PhidgetGPSHandle ch;

	ch = (PhidgetGPSHandle)phid;

	if(ch->heading != PUNK_DBL &&
	  ch->velocity != PUNK_DBL)
		FIRECH(ch, PhidgetGPS, HeadingChange, ch->heading, ch->velocity);
	if(ch->latitude != PUNK_DBL &&
	  ch->longitude != PUNK_DBL &&
	  ch->altitude != PUNK_DBL)
		FIRECH(ch, PhidgetGPS, PositionChange, ch->latitude, ch->longitude, ch->altitude);
	if(ch->positionFixState != PUNK_BOOL)
		FIRECH(ch, PhidgetGPS, PositionFixStateChange, ch->positionFixState);

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {
	PhidgetGPSHandle ch;

	ch = (PhidgetGPSHandle)phid;

	if(ch->heading == PUNK_DBL ||
	  ch->velocity == PUNK_DBL)
		return (PFALSE);
	if(ch->latitude == PUNK_DBL ||
	  ch->longitude == PUNK_DBL ||
	  ch->altitude == PUNK_DBL)
		return (PFALSE);
	if(ch->positionFixState == PUNK_BOOL)
		return (PFALSE);

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetGPS));
}

static PhidgetReturnCode CCONV
_create(PhidgetGPSHandle *phidp) {

	CHANNELCREATE_BODY(GPS, PHIDCHCLASS_GPS);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_delete(PhidgetGPSHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetGPS_getAltitude(PhidgetGPSHandle ch, double *altitude) {

	TESTPTR_PR(ch);
	TESTPTR_PR(altitude);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);
	TESTATTACHED_PR(ch);

	*altitude = ch->altitude;
	if (ch->altitude == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_getDate(PhidgetGPSHandle ch, PhidgetGPS_Date *date) {

	TESTPTR_PR(ch);
	TESTPTR_PR(date);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);
	TESTATTACHED_PR(ch);

	*date = ch->date;
	if (ch->dateValid == PFALSE)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_getHeading(PhidgetGPSHandle ch, double *heading) {

	TESTPTR_PR(ch);
	TESTPTR_PR(heading);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);
	TESTATTACHED_PR(ch);

	*heading = ch->heading;
	if (ch->heading == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_getLatitude(PhidgetGPSHandle ch, double *latitude) {

	TESTPTR_PR(ch);
	TESTPTR_PR(latitude);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);
	TESTATTACHED_PR(ch);

	*latitude = ch->latitude;
	if (ch->latitude == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_getLongitude(PhidgetGPSHandle ch, double *longitude) {

	TESTPTR_PR(ch);
	TESTPTR_PR(longitude);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);
	TESTATTACHED_PR(ch);

	*longitude = ch->longitude;
	if (ch->longitude == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_getNMEAData(PhidgetGPSHandle ch, PhidgetGPS_NMEAData *NMEAData) {

	TESTPTR_PR(ch);
	TESTPTR_PR(NMEAData);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);
	TESTATTACHED_PR(ch);

	*NMEAData = ch->NMEAData;
	if (ch->NMEADataValid == PFALSE)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_getPositionFixState(PhidgetGPSHandle ch, int *positionFixState) {

	TESTPTR_PR(ch);
	TESTPTR_PR(positionFixState);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);
	TESTATTACHED_PR(ch);

	*positionFixState = ch->positionFixState;
	if (ch->positionFixState == (int)PUNK_BOOL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_getTime(PhidgetGPSHandle ch, PhidgetGPS_Time *time) {

	TESTPTR_PR(ch);
	TESTPTR_PR(time);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);
	TESTATTACHED_PR(ch);

	*time = ch->time;
	if (ch->timeValid == PFALSE)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_getVelocity(PhidgetGPSHandle ch, double *velocity) {

	TESTPTR_PR(ch);
	TESTPTR_PR(velocity);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);
	TESTATTACHED_PR(ch);

	*velocity = ch->velocity;
	if (ch->velocity == (double)PUNK_DBL)
		return (PHID_RETURN(EPHIDGET_UNKNOWNVAL));
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_setOnHeadingChangeHandler(PhidgetGPSHandle ch, PhidgetGPS_OnHeadingChangeCallback fptr,
  void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->HeadingChange = NULL;
		MEMORY_BARRIER();
		ch->HeadingChangeCtx = NULL;
	} else {
		ch->HeadingChangeCtx = ctx;
		MEMORY_BARRIER();
		ch->HeadingChange = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetGPS_setOnPositionChangeHandler(PhidgetGPSHandle ch, PhidgetGPS_OnPositionChangeCallback fptr,
  void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);

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
PhidgetGPS_setOnPositionFixStateChangeHandler(PhidgetGPSHandle ch,
  PhidgetGPS_OnPositionFixStateChangeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_GPS);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->PositionFixStateChange = NULL;
		MEMORY_BARRIER();
		ch->PositionFixStateChangeCtx = NULL;
	} else {
		ch->PositionFixStateChangeCtx = ctx;
		MEMORY_BARRIER();
		ch->PositionFixStateChange = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}
