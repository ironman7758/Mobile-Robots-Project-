/* Generated: Mon Mar 24 2025 13:18:57 GMT-0600 (Mountain Daylight Time) */

#include "device/irdevice.h"
static void CCONV PhidgetIR_errorHandler(PhidgetChannelHandle ch, Phidget_ErrorEventCode code);
static void CCONV PhidgetIR_free(PhidgetChannelHandle *ch);
static PhidgetReturnCode CCONV PhidgetIR_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetIR_setStatus(PhidgetChannelHandle phid, BridgePacket *bp);
static PhidgetReturnCode CCONV PhidgetIR_getStatus(PhidgetChannelHandle phid, BridgePacket **bp);
static PhidgetReturnCode CCONV PhidgetIR_initAfterOpen(PhidgetChannelHandle phid);
static PhidgetReturnCode CCONV PhidgetIR_setDefaults(PhidgetChannelHandle phid);
static void CCONV PhidgetIR_fireInitialEvents(PhidgetChannelHandle phid);
static int CCONV PhidgetIR_hasInitialState(PhidgetChannelHandle phid);

struct _PhidgetIR {
	struct _PhidgetChannel phid;
	PhidgetIR_CodeInfo lastCodeInfo;
	PhidgetIR_CodeInfo lastLearnedCodeInfo;
	char lastCodeStr[33];
	char lastLearnedCodeStr[33];
	int lastCodeKnown;
	int lastLearnedCodeKnown;
	volatile PhidgetIR_OnCodeCallback Code;
	volatile void *CodeCtx;
	volatile PhidgetIR_OnLearnCallback Learn;
	volatile void *LearnCtx;
	volatile PhidgetIR_OnRawDataCallback RawData;
	volatile void *RawDataCtx;
};

static PhidgetReturnCode CCONV
_setStatus(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetIRHandle ch;
	int nsversion;

	ch = (PhidgetIRHandle)phid;

	nsversion = getBridgePacketUInt32ByName(bp, "_class_version_");
	if (nsversion != 1) {
		loginfo("%"PRIphid": server/client class version mismatch: %d != 1 - functionality may be limited.", phid, nsversion);
	}

	if(hasBridgePacketByName(bp, "lastCodeStr"))
		memcpy(&ch->lastCodeStr, getBridgePacketUInt8ArrayByName(bp, "lastCodeStr"), sizeof (char) * 33);
	if(hasBridgePacketByName(bp, "lastLearnedCodeStr"))
		memcpy(&ch->lastLearnedCodeStr, getBridgePacketUInt8ArrayByName(bp, "lastLearnedCodeStr"),
	  sizeof (char) * 33);
	if(hasBridgePacketByName(bp, "lastCodeKnown"))
		ch->lastCodeKnown = getBridgePacketInt32ByName(bp, "lastCodeKnown");
	if(hasBridgePacketByName(bp, "lastLearnedCodeKnown"))
		ch->lastLearnedCodeKnown = getBridgePacketInt32ByName(bp, "lastLearnedCodeKnown");

	return (EPHIDGET_OK);
}

static PhidgetReturnCode CCONV
_getStatus(PhidgetChannelHandle phid, BridgePacket **bp) {
	PhidgetIRHandle ch;

	ch = (PhidgetIRHandle)phid;

	return (createBridgePacket(bp, BP_SETSTATUS, 5, "_class_version_=%u"
	  ",lastCodeStr=%33R"
	  ",lastLearnedCodeStr=%33R"
	  ",lastCodeKnown=%d"
	  ",lastLearnedCodeKnown=%d"
	  ,1 /* class version */
	  ,ch->lastCodeStr
	  ,ch->lastLearnedCodeStr
	  ,ch->lastCodeKnown
	  ,ch->lastLearnedCodeKnown
	));
}

static PhidgetReturnCode CCONV
_bridgeInput(PhidgetChannelHandle phid, BridgePacket *bp) {
	PhidgetReturnCode res;

	res = EPHIDGET_OK;

	switch (bp->vpkt) {
	case BP_TRANSMIT:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_TRANSMITRAW:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	case BP_TRANSMITREPEAT:
		res = DEVBRIDGEINPUT(phid, bp);
		break;
	default:
		logerr("%"PRIphid": unsupported bridge packet:0x%x", phid, bp->vpkt);
		res = EPHIDGET_UNSUPPORTED;
	}

	return (res);
}

static PhidgetReturnCode CCONV
_initAfterOpen(PhidgetChannelHandle phid) {
	PhidgetIRHandle ch;
	PhidgetReturnCode ret;

	TESTPTR(phid);
	ch = (PhidgetIRHandle)phid;

	ret = EPHIDGET_OK;


	switch (phid->UCD->uid) {
	case PHIDCHUID_1055_IR_100:
		break;
	case PHIDCHUID_1055_IR_200_USB:
		break;
	case PHIDCHUID_1055_IR_200_VINT:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	memset(&ch->lastCodeInfo, 0, sizeof (ch->lastCodeInfo));
	memset(&ch->lastLearnedCodeInfo, 0, sizeof (ch->lastLearnedCodeInfo));
	memset(ch->lastCodeStr, 0, sizeof (char) * 33);
	memset(ch->lastLearnedCodeStr, 0, sizeof (char) * 33);
	ch->lastCodeKnown = 0;
	ch->lastLearnedCodeKnown = 0;

	return (ret);
}

static PhidgetReturnCode CCONV
_setDefaults(PhidgetChannelHandle phid) {
	PhidgetReturnCode ret;

	TESTPTR(phid);

	ret = EPHIDGET_OK;

	switch (phid->UCD->uid) {
	case PHIDCHUID_1055_IR_100:
		break;
	case PHIDCHUID_1055_IR_200_USB:
		break;
	case PHIDCHUID_1055_IR_200_VINT:
		break;
	default:
		MOS_PANIC("Unsupported Channel");
	}

	return (ret);
}

static void CCONV
_fireInitialEvents(PhidgetChannelHandle phid) {

}

static int CCONV
_hasInitialState(PhidgetChannelHandle phid) {

	return (PTRUE);
}

static void CCONV
_free(PhidgetChannelHandle *ch) {

	mos_free(*ch, sizeof (struct _PhidgetIR));
}

static PhidgetReturnCode CCONV
_create(PhidgetIRHandle *phidp) {

	CHANNELCREATE_BODY(IR, PHIDCHCLASS_IR);
	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetIR_delete(PhidgetIRHandle *phidp) {

	return (Phidget_delete((PhidgetHandle *)phidp));
}

API_PRETURN
PhidgetIR_transmitRepeat(PhidgetIRHandle ch) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);
	TESTATTACHED_PR(ch);

	return bridgeSendToDevice((PhidgetChannelHandle)ch, BP_TRANSMITREPEAT, NULL, NULL, 0, NULL); 
}

API_PRETURN
PhidgetIR_setOnCodeHandler(PhidgetIRHandle ch, PhidgetIR_OnCodeCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->Code = NULL;
		MEMORY_BARRIER();
		ch->CodeCtx = NULL;
	} else {
		ch->CodeCtx = ctx;
		MEMORY_BARRIER();
		ch->Code = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetIR_setOnLearnHandler(PhidgetIRHandle ch, PhidgetIR_OnLearnCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->Learn = NULL;
		MEMORY_BARRIER();
		ch->LearnCtx = NULL;
	} else {
		ch->LearnCtx = ctx;
		MEMORY_BARRIER();
		ch->Learn = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}

API_PRETURN
PhidgetIR_setOnRawDataHandler(PhidgetIRHandle ch, PhidgetIR_OnRawDataCallback fptr, void *ctx) {

	TESTPTR_PR(ch);
	TESTCHANNELCLASS_PR(ch, PHIDCHCLASS_IR);

	MEMORY_BARRIER();
	if (fptr == NULL) {
		ch->RawData = NULL;
		MEMORY_BARRIER();
		ch->RawDataCtx = NULL;
	} else {
		ch->RawDataCtx = ctx;
		MEMORY_BARRIER();
		ch->RawData = fptr;
	}
	MEMORY_BARRIER();

	return (EPHIDGET_OK);
}
