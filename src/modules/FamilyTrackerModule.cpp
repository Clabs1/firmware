#include "FamilyTrackerModule.h"
#include "GPS.h"
#include "NodeDB.h"
#include "NodeInfoModule.h"
#include "main.h" // screen
#include "gps/RTC.h"
#include "gps/GeoCoord.h"
#include "buzz/buzz.h"

// ENH-009 developer debug mode: compiled in only with -DFAMILY_DEBUG_CHAT=1
// (see family_common.ini debug envs). Everywhere else FT_DEBUG(...) vanishes.
#ifdef FAMILY_DEBUG_CHAT
#define FT_DEBUG(...) ftDebug(__VA_ARGS__)
#else
#define FT_DEBUG(...)
#endif
#if HAS_SCREEN
#include "graphics/draw/NotificationRenderer.h"
#endif
#include <cstdarg>
#include <cstring>
#include <cmath>
// __attribute__((used)) on the global forces whole-image LTO to retain the
// construction call (familyTrackerModule = new FamilyTrackerModule()) in
// setupModules(), otherwise the side-effect registration into MeshModule::modules
// is elided and the module silently never runs.
__attribute__((used)) FamilyTrackerModule *familyTrackerModule;

// LTO guard: returns the module instance so setupModules()' construction call is
// retained under whole-image LTO (nrf52_lto.py). Referenced from main.cpp.
__attribute__((used)) FamilyTrackerModule *getFamilyTrackerModule()
{
    return familyTrackerModule;
}

// Preselected "on the way" response messages (SPEC §34A). Index carried in the
// PARENT_ON_WAY payload (byte 7). Must match FAMILYTRACKER_ON_WAY_COUNT.
static const char *const familyTrackerOnWayMessages[FAMILYTRACKER_ON_WAY_COUNT] = {
    "On my way!",
    "Read. Coming now!",
    "Calling for help",
    "Stay put",
};

FamilyTrackerModule::FamilyTrackerModule()
    : SinglePortModule("FamilyTracker", meshtastic_PortNum_PRIVATE_APP),
      concurrency::OSThread("FamilyTracker", 5000) // tick every 5s
{
    if (inputBroker)
        inputObserver.observe(inputBroker);
}

bool FamilyTrackerModule::isChild() const
{
    return IS_ONE_OF(config.device.role, meshtastic_Config_DeviceConfig_Role_TRACKER,
                     meshtastic_Config_DeviceConfig_Role_TAK_TRACKER);
}

bool FamilyTrackerModule::isParent() const
{
    return !isChild(); // CLIENT / ROUTER / ROUTER_CLIENT etc.
}

bool FamilyTrackerModule::isBase() const
{
    // CLIENT_BASE is THE stationary camp/car role (app-visible). ROUTER /
    // ROUTER_CLIENT stay recognised for older provisioned units.
    return IS_ONE_OF(config.device.role, meshtastic_Config_DeviceConfig_Role_CLIENT_BASE,
                     meshtastic_Config_DeviceConfig_Role_ROUTER, meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT);
}

void FamilyTrackerModule::setup()
{
    // A normal-message tone re-plays while the external-notification nag window
    // is open (default 15s > the ~1.8s default RTTTL melody). Shrink it below
    // the melody length so a message tone plays once, not ~3x.
    // One play-through of the phone ringtone: nag window must cover the whole
    // tune (~5 s) so the nRF52 RTTTL player never restarts it (ENH-006).
    moduleConfig.external_notification.nag_timeout = 10;
    if (isBase()) {
        // Base/relay node: silence the generic message/bell tones. The family
        // emergency tones (panic/lost-child) bypass ExternalNotification and
        // still play.
        moduleConfig.external_notification.alert_message_buzzer = false;
        moduleConfig.external_notification.alert_bell_buzzer = false;
    }
LOG_WARN("FamilyTrackerModule: armed (role=%s)", isChild() ? "child" : isBase() ? "base" : "parent");
    FT_DEBUG("armed role=%s hw=%d v%s node=0x%08x", isChild() ? "child" : isBase() ? "base" : "parent",
             (int)HW_VENDOR, xstr(APP_VERSION_SHORT), (unsigned)nodeDB->getNodeNum());
}

void FamilyTrackerModule::buzzerBeep(bool ack)
{
    if (ack) {
        playBoop();
    } else {
        playBeep();
    }
}

void FamilyTrackerModule::markParentHeard()
{
    // Any family packet authored by a non-tracker proves parent contact.
    // Clears a latched "parent missing" alert so it can fire again later.
    lastParentHeardMs = millis();
    parentHeardEver = true;
    parentMissingAlerted = false;
}

void FamilyTrackerModule::triggerFreshFix()
{
#if HAS_GPS
    if (gps)
        gps->setPowerState(GPSPowerState::GPS_ACTIVE);
#endif
}

void FamilyTrackerModule::fillBestPosition(meshtastic_PositionLite *pos, bool *hasPos, bool *stale, uint8_t *ageMin)
{
    *hasPos = false;
    *stale = false;
    *ageMin = 0;
    memset(pos, 0, sizeof(*pos));

    const meshtastic_NodeInfoLite *self = nodeDB->getMeshNode(nodeDB->getNodeNum());
    if (!nodeDB->hasValidPosition(self))
        return;

    bool latLonValid = (localPosition.latitude_i != 0 || localPosition.longitude_i != 0);
    if (!latLonValid)
        return;

    pos->latitude_i = localPosition.latitude_i;
    pos->longitude_i = localPosition.longitude_i;
    pos->altitude = localPosition.altitude;

    uint32_t fixTime = localPosition.timestamp > 0 ? localPosition.timestamp : localPosition.time;
    // Age math in UTC epoch (BUG-006): localPosition.timestamp is UTC, so
    // comparing it against a LOCAL clock (BST = UTC+1) skews the age by the TZ
    // offset. Local time is a display concern only.
    uint32_t nowSecs = getValidTime(RTCQuality::RTCQualityDevice, false);
    if (fixTime > 0 && nowSecs > fixTime) {
        uint32_t ageSecs = nowSecs - fixTime;
        if (ageSecs > FAMILYTRACKER_POSITION_FRESH_SECS) {
            *stale = true;
            *ageMin = (uint8_t)(ageSecs / 60);
        }
    }
    *hasPos = true;
}

void FamilyTrackerModule::renderPanicAlert(NodeNum from, uint32_t eventId, uint32_t eventTs,
                                           const meshtastic_PositionLite &pos, bool stale, uint8_t ageMin, bool hasPos)
{
    // Child identity (SPEC §18A multi-child): name from the child's nodedb entry.
    const char *childName = nullptr;
    const meshtastic_NodeInfoLite *child = nodeDB->getMeshNode(from);
    if (child && child->long_name[0]) {
        childName = child->long_name;
    } else if (child && child->short_name[0]) {
        childName = child->short_name;
    }
    char nameBuf[16];
    if (!childName) {
        snprintf(nameBuf, sizeof(nameBuf), "0x%04x", (uint16_t)from);
        childName = nameBuf;
    }

    // Urgent on-screen banner (SPEC §34A "child name PANIC button pressed"):
    // the text-message path would only show a generic "New Message from X", so
    // surface the panic explicitly on display hardware.
    if (screen) {
        char banner[80];
        snprintf(banner, sizeof(banner), "%s PANIC button pressed", childName);
        screen->showSimpleBanner(banner, 15000);
    }

    // Panic time: the child's authoritative event timestamp (UTC epoch),
    // rendered in OUR local timezone (BUG-005/006 - immutable child event data;
    // every parent shows the same instant).
    char timeStr[8] = "--:--";
    if (eventTs) {
        uint32_t utcNow = getValidTime(RTCQuality::RTCQualityDevice, false);
        uint32_t localNow = getValidTime(RTCQuality::RTCQualityDevice, true);
        uint32_t localEventTs = eventTs + (uint32_t)((int32_t)localNow - (int32_t)utcNow);
        uint8_t hh = (localEventTs / 3600) % 24;
        uint8_t mm = (localEventTs / 60) % 60;
        snprintf(timeStr, sizeof(timeStr), "%02u:%02u", hh, mm);
    }

    // Distance + bearing from the parent's own position (SPEC §35 geo distance,
    // never RSSI/hops) to the child's reported position. Parents may only ADD
    // this presentation data - never rewrite the child's event metadata.
    float distM = -1.0f;
    int bearingDeg = -1;
    const meshtastic_NodeInfoLite *self = nodeDB->getMeshNode(nodeDB->getNodeNum());
    bool haveSelf = self && nodeDB->hasValidPosition(self);
    bool haveChild = hasPos && (pos.latitude_i != 0 || pos.longitude_i != 0);
    if (haveSelf && haveChild) {
        float selfLat = localPosition.latitude_i * 1e-7;
        float selfLon = localPosition.longitude_i * 1e-7;
        distM = GeoCoord::latLongToMeter(selfLat, selfLon, pos.latitude_i * 1e-7, pos.longitude_i * 1e-7);
        bearingDeg = (int)GeoCoord::bearing(selfLat, selfLon, pos.latitude_i * 1e-7, pos.longitude_i * 1e-7);
        if (bearingDeg < 0)
            bearingDeg += 360;
    }

    // Age suffix: "(x mins ago)" or "(stale, x mins ago)" / "(position unknown)".
    char ageStr[40];
    if (!haveChild) {
        snprintf(ageStr, sizeof(ageStr), "position unknown");
    } else if (stale && ageMin > 0) {
        snprintf(ageStr, sizeof(ageStr), "%u mins ago", ageMin);
    } else {
        snprintf(ageStr, sizeof(ageStr), "fresh");
    }

    if (distM >= 0 && bearingDeg >= 0) {
        char distStr[24];
        if (distM < 1000.0f)
            snprintf(distStr, sizeof(distStr), "%.0f m", distM);
        else
            snprintf(distStr, sizeof(distStr), "%.1f km", distM / 1000.0f);
        // Log-only: the child already broadcast ONE authoritative human-readable
        // text. A per-parent rebroadcast here is exactly what BUG-004 forbids.
        LOG_WARN("FamilyTracker: PANIC event=%u ts=%u from %s (0x%04x) - %.1f m away %d deg (%s)", eventId, eventTs,
                 childName, (uint16_t)from, distM, bearingDeg, ageStr);
    } else {
        LOG_WARN("FamilyTracker: PANIC event=%u ts=%u from %s (0x%04x) - %s", eventId, eventTs, childName, (uint16_t)from,
                 ageStr);
    }
}

void FamilyTrackerModule::sendMessage(uint8_t msgType, uint32_t eventId, bool hasPos, bool stale, uint8_t ageMin)
{
    sendMessageTo(msgType, eventId, hasPos, stale, ageMin, NODENUM_BROADCAST);
}

void FamilyTrackerModule::sendMessageTo(uint8_t msgType, uint32_t eventId, bool hasPos, bool stale, uint8_t ageMin,
                                        NodeNum to)
{
    meshtastic_PositionLite pos;
    if (hasPos) {
        // (re)fetch best position
        bool hp, st;
        uint8_t am;
        fillBestPosition(&pos, &hp, &st, &am);
        hasPos = hp;
        if (st)
            stale = true;
        if (am)
            ageMin = am;
    }

    // Event timestamp: UTC epoch at send time, captured by the ORIGINATOR so the
    // event data stays immutable across the mesh (BUG-005, ARCH §1/§5).
    uint32_t eventTs = getValidTime(RTCQuality::RTCQualityDevice, false);

    uint8_t payload[22] = {0};
    uint8_t flags = 0;
    if (hasPos)
        flags |= FAMILYTRACKER_FLAG_HAS_POS;
    if (stale)
        flags |= FAMILYTRACKER_FLAG_POS_STALE;

    payload[0] = FAMILYTRACKER_PROTOCOL_VERSION;
    payload[1] = msgType;
    payload[2] = eventId & 0xFF;
    payload[3] = (eventId >> 8) & 0xFF;
    payload[4] = (eventId >> 16) & 0xFF;
    payload[5] = (eventId >> 24) & 0xFF;
    payload[6] = flags;
    payload[7] = ageMin;
    payload[8] = eventTs & 0xFF;
    payload[9] = (eventTs >> 8) & 0xFF;
    payload[10] = (eventTs >> 16) & 0xFF;
    payload[11] = (eventTs >> 24) & 0xFF;
    size_t len = 12;
    if (hasPos) {
        payload[12] = pos.latitude_i & 0xFF;
        payload[13] = (pos.latitude_i >> 8) & 0xFF;
        payload[14] = (pos.latitude_i >> 16) & 0xFF;
        payload[15] = (pos.latitude_i >> 24) & 0xFF;
        payload[16] = pos.longitude_i & 0xFF;
        payload[17] = (pos.longitude_i >> 8) & 0xFF;
        payload[18] = (pos.longitude_i >> 16) & 0xFF;
        payload[19] = (pos.longitude_i >> 24) & 0xFF;
        payload[20] = pos.altitude & 0xFF;
        payload[21] = (pos.altitude >> 8) & 0xFF;
        len = 22;
    }

    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p)
        return;
    p->to = to;
    p->decoded.payload.size = len;
    memcpy(p->decoded.payload.bytes, payload, len);

    service->sendToMesh(p, RX_SRC_LOCAL, true);

    if (msgType == FAMILYTRACKER_MSG_PANIC)
        playPanicCall(); // SENT tone - three short "call" notes (SPEC §33)

    LOG_INFO("FamilyTracker: TX msg=%u event=%u ts=%u to=%04x%s%s", msgType, eventId, eventTs, (uint16_t)to,
             hasPos ? " pos" : "", stale ? " (stale)" : "");
}

void FamilyTrackerModule::sendAck(uint32_t eventId, NodeNum to)
{
    // Targeted ACK to the child that raised the panic (multi-Child §18A):
    // prevents a broadcast ACK from being matched by a different child that
    // happened to use the same eventId (each child numbers events locally).
    sendMessageTo(FAMILYTRACKER_MSG_PANIC_ACK, eventId, false, false, 0, to);
}

void FamilyTrackerModule::sendPanic()
{
    uint32_t nowMs = millis();
    // Cooldown/retrigger (BUG-008, ARCH §2): a fresh panic requires a new event
    // ID and is throttled so a held/rapid button can't flood the mesh.
    if (panicState != FamilyPanicState::NORMAL && (nowMs - lastPanicSentMs < FAMILYTRACKER_PANIC_COOLDOWN_MS)) {
        LOG_INFO("FamilyTracker: panic suppressed (cooldown %lu ms)", (unsigned long)(nowMs - lastPanicSentMs));
        return;
    }
    bool firstTrigger = (panicState != FamilyPanicState::PANIC_ACTIVE);
    if (panicState == FamilyPanicState::PANIC_CLEARED)
        panicState = FamilyPanicState::NORMAL;
    uint32_t eventId = nextEventId++;
    lastPanicEventId = eventId;
    lastPanicSentMs = nowMs;
    panicState = FamilyPanicState::PANIC_ACTIVE;
    sendMessage(FAMILYTRACKER_MSG_PANIC, eventId, true, false, 0);

    // ONE authoritative human-readable broadcast (BUG-004): the child owns the
    // panic narrative; parents never regenerate it. Deferred to runOnce because
    // sendPanic can run inside the radio RX path (PANIC_TRIGGER).
    char timeStr[8] = "--:--";
    uint32_t localNow = getValidTime(RTCQuality::RTCQualityDevice, true);
    if (localNow) {
        uint8_t hh = (localNow / 3600) % 24;
        uint8_t mm = (localNow / 60) % 60;
        snprintf(timeStr, sizeof(timeStr), "%02u:%02u", hh, mm);
    }
    // Explicit location status (BUG-018): the announcement always states where
    // we are (or were) - or that we don't know. Same flags the PANIC datagram
    // carries, so chat and protocol never disagree.
    meshtastic_PositionLite pos;
    bool hasPos = false, stale = false;
    uint8_t ageMin = 0;
    fillBestPosition(&pos, &hasPos, &stale, &ageMin);
    char locStr[40];
    if (!hasPos)
        snprintf(locStr, sizeof(locStr), "location unknown");
    else if (stale && ageMin > 0)
        snprintf(locStr, sizeof(locStr), "location %u mins old", ageMin);
    else
        snprintf(locStr, sizeof(locStr), "location fresh");
    queueTextAlert("%s PANIC button pressed at %s (%s)", owner.long_name, timeStr, locStr);
    FT_DEBUG("panic tx event=%u gps=%s", (unsigned)eventId, locStr);

    // Sibling children receive RETURN_TO_PARENT once (BUG-009/ARCH §3) so the
    // whole group regroups, not just the panicking child. Datagram only -
    // ENH-007: no second chat message from this child.
    if (firstTrigger)
        sendComeBackRegroup();
}

void FamilyTrackerModule::notifySelfText(const char *fmt, ...)
{
    char msg[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    meshtastic_MeshPacket *p = allocDataPacket();
    if (p) {
        p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        p->to = nodeDB->getNodeNum();
        p->decoded.payload.size = strnlen(msg, sizeof(msg));
        memcpy(p->decoded.payload.bytes, msg, p->decoded.payload.size);
        service->sendToMesh(p, RX_SRC_LOCAL, true);
    }
}

void FamilyTrackerModule::sendOnWay(uint32_t eventId, uint8_t presetIndex, NodeNum to, bool announce)
{
    if (presetIndex >= FAMILYTRACKER_ON_WAY_COUNT)
        presetIndex = 0;
    const char *msg = familyTrackerOnWayMessages[presetIndex];

    // 1) Human-readable Family Channel text (SPEC §34A) so the whole family
    //    (and any parent console/app) sees who responded and what they said.
    //    Deferred: ON_WAY_TRIGGER calls us from the RX path.
    //    announce=false when re-firing another parent's response - no dupe text.
    char buf[128];
    snprintf(buf, sizeof(buf), "%s: %s", owner.short_name, msg);
    if (announce)
        queueTextAlert("%s", buf);

    // 2) PARENT_ON_WAY datagram, targeted to the child: distinct tone + on-screen
    //    message at the child. presetIndex rides in the ageMin byte.
    sendMessageTo(FAMILYTRACKER_MSG_PARENT_ON_WAY, eventId, false, false, presetIndex, to);
    LOG_INFO("FamilyTracker: PARENT_ON_WAY event=%u to=%04x preset=%u '%s'", eventId, (uint16_t)to, presetIndex, msg);
}

void FamilyTrackerModule::sendComeBack()
{
    sendComeBackMsg(NODENUM_BROADCAST, true);
}

void FamilyTrackerModule::sendComeBackTo(NodeNum target)
{
    const meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(target);
    const char *cn = (n && n->long_name[0]) ? n->long_name
                     : (n && n->short_name[0]) ? n->short_name
                                               : "Child";
    LOG_WARN("FamilyTracker: COME BACK targeted at 0x%08x (%s)", (unsigned)target, cn);
    sendComeBackMsg(target, true);
}

void FamilyTrackerModule::sendComeBackRegroup()
{
    // Child-originated regroup (ENH-007): when THIS child panics, siblings get
    // the return-to-parent datagram only - no family-chat text. The panicking
    // child's own panic announcement is the single authoritative narrative
    // (BUG-004); a second chat message just clutters the group.
    sendComeBackMsg(NODENUM_BROADCAST, false);
}

void FamilyTrackerModule::sendComeBackMsg(NodeNum to, bool announce)
{
    // Parent -> children: "come back now". Datagram drives the Mario tune +
    // banner at each child; the optional group-chat text keeps humans in the
    // loop ("COME BACK: " is matched by isFamilyAlertText so receivers consume
    // it without the generic tone).
    sendMessageTo(FAMILYTRACKER_MSG_COME_BACK, 0, false, false, 0, to);
    if (announce) {
        if (to == NODENUM_BROADCAST)
            queueTextAlert("COME BACK: %s says come back now", owner.short_name);
        else {
            const meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(to);
            const char *cn = (n && n->long_name[0]) ? n->long_name
                             : (n && n->short_name[0]) ? n->short_name
                                                       : "Child";
            queueTextAlert("COME BACK: %s says %s come back now", owner.short_name, cn);
        }
    }
    LOG_INFO("FamilyTracker: COME BACK to=%04x announced=%d", (uint16_t)to, announce);
}

void FamilyTrackerModule::sendFound()
{
    // Parent concludes a lost/panic search: broadcast a stand-down so EVERY node
    // clears its state and plays the "level complete" success tone. The child that
    // was actually active announces "FOUND" once (see the CANCEL handler).
    sendMessageTo(FAMILYTRACKER_MSG_CANCEL, 0, false, false, 0, NODENUM_BROADCAST);
    lostDeclaredByUs.clear();   // global stand-down clears our declarations too
    parentAlertNodes.clear();   // and the persistent panic indication (ENH-001)
    LOG_WARN("FamilyTracker: FOUND stand-down broadcast by parent");
}

bool FamilyTrackerModule::sendFoundTo(NodeNum target)
{
    // Targeted stand-down ("found bob"): only that child clears; siblings are
    // untouched. The child announces its own FOUND text (attribution-neutral).
    sendMessageTo(FAMILYTRACKER_MSG_CANCEL, 0, false, false, 0, target);
    parentAlertNodes.erase(target); // targeted stand-down ends this child's alert (ENH-001)
    LOG_WARN("FamilyTracker: FOUND targeted at 0x%08x", (unsigned)target);
    return true;
}

size_t FamilyTrackerModule::stillMissingNames(char *out, size_t outLen)
{
    size_t used = 0;
    out[0] = '\0';
    for (NodeNum num : lostDeclaredByUs) {
        const meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(num);
        const char *cn = (n && n->long_name[0]) ? n->long_name : (n && n->short_name[0]) ? n->short_name : nullptr;
        if (!cn)
            continue;
        size_t add = strlen(cn) + 2;
        if (used + add >= outLen)
            break;
        if (used)
            out[used++] = ',', out[used++] = ' ';
        used += snprintf(out + used, outLen - used, "%s", cn);
    }
    return used;
}

// Whole-word command match: the command must be followed by a non-alphanumeric
// character (space, punctuation, or end) so "lost" doesn't fire on "lostal".
static bool matchCommand(const char *text, const char *cmd)
{
    size_t n = strlen(cmd);
    if (strncasecmp(text, cmd, n) != 0)
        return false;
    char c = text[n];
    return !(c >= 'a' && c <= 'z') && !(c >= 'A' && c <= 'Z') && !(c >= '0' && c <= '9');
}

// Forward-declare (defined after handleTextCommand).
static const char *findNoCase(const char *haystack, const char *needle);

// Map a parent's free-text response onto one of the preselected on-way presets.
// Returns -1 when the text isn't an on-way response at all.
static int onWayPresetFromText(const char *buf)
{
    if (findNoCase(buf, "on my way"))
        return 0;
    if (findNoCase(buf, "coming now") || findNoCase(buf, "coming"))
        return 1;
    if (findNoCase(buf, "calling for help") || findNoCase(buf, "calling help"))
        return 2;
    if (findNoCase(buf, "stay put") || findNoCase(buf, "stay where"))
        return 3;
    return -1;
}

// Extract the child name after a command, trimming surrounding whitespace and
// trailing punctuation ("lost Alice!" -> "Alice").
static void extractName(const char *src, char *out, size_t outLen)
{
    out[0] = '\0';
    if (!src || !*src) // never read past the terminator (offset bugs would
        return;        // otherwise scrape stale stack bytes as a "name")
    while (*src == ' ' || *src == '\t')
        src++;
    size_t n = 0;
    while (src[n] && n < outLen - 1) {
        char c = src[n];
        if (c == '.' || c == '!' || c == '?' || c == ',' || c == ';' || c == '\n' || c == '\r')
            break;
        out[n++] = c;
    }
    out[n] = '\0';
    while (n > 0 && (out[n - 1] == ' ' || out[n - 1] == '\t'))
        out[--n] = '\0';
}

bool FamilyTrackerModule::handleFamilyCommand(const char *text)
{
    if (!text || !isParent())
        return false;

    // Trim leading whitespace, then match whole commands case-insensitively.
    while (*text == ' ' || *text == '\t')
        text++;

    if (matchCommand(text, "come back")) {
        // "come back <name>" targets one child; bare "come back" is the regroup
        // broadcast to everyone.
        char name[32];
        extractName(text + 9, name, sizeof(name));
        if (name[0]) {
            NodeNum target = findChildByName(name);
            if (target) {
                sendComeBackTo(target);
                return true;
            }
            unknownChildError(name);
            return true;
        }
        sendComeBack();
        return true;
    }
        if (matchCommand(text, "found")) {
        // "found <name>" clears ONE child; bare "found" is the global stand-down.
        char name[32];
        extractName(text + 5, name, sizeof(name));
        if (name[0]) {
            NodeNum target = findChildByName(name);
            if (target) {
                sendFoundTo(target);
                if (!isBase())
                    playFoundMelody(); // parents also hear the success tone
                lostDeclaredByUs.erase(target);
                char still[96];
                if (stillMissingNames(still, sizeof(still)))
                    queueTextAlert("STILL MISSING: %s", still); // deferred - same policy as every RX-path text
                return true;
            }
            unknownChildError(name);
            return true;
        }
        sendFound();
        return true;
    }
    if (matchCommand(text, "lost")) {
        // "lost <name>" (canned) - name follows "lost".
        char name[32];
        extractName(text + 4, name, sizeof(name));
        if (name[0]) {
            NodeNum target = findChildByName(name);
            if (target) {
                sendLostChild(target);
                return true;
            }
            unknownChildError(name);
            return true;
        }
        pickLostChild(); // bare "lost": pick from the checked-in children
        return true;
    }

    if (matchCommand(text, "find")) {
        // "find <name>" - loud repeating tone on that node (dropped in grass).
        // Falls back to the whole nodedb so a base/car unit can chirp too.
        char name[32];
        extractName(text + 4, name, sizeof(name));
        if (name[0]) {
            NodeNum target = findChildByName(name);
            if (!target)
                target = findAnyNodeByName(name);
            if (target) {
                sendFindSound(target);
                return true;
            }
            unknownChildError(name);
            return true;
        }
        playErrorTone();
#if HAS_SCREEN
        if (screen)
            screen->showSimpleBanner("Usage: find <name>", 5000);
#endif
        return true;
    }
    return false;
}

NodeNum FamilyTrackerModule::findChildByName(const char *name)
{
    if (!name || !*name)
        return 0;

    // Only children that have actually checked in (heartbeat) are considered -
    // the nodedb may hold stale or non-family TRACKER nodes.
    std::vector<NodeNum> checkedIn;
    {
        concurrency::LockGuard guard(&childSeenLock);
        for (const auto &kv : childLastSeenMs)
            checkedIn.push_back(kv.first);
    }
    for (NodeNum num : checkedIn) {
        const meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(num);
        if (!n)
            continue;
        if ((n->long_name[0] && strcasecmp(n->long_name, name) == 0) ||
            (n->short_name[0] && strcasecmp(n->short_name, name) == 0))
            return num;
    }
    return 0;
}

NodeNum FamilyTrackerModule::findAnyNodeByName(const char *name)
{
    if (!name || !*name)
        return 0;
    for (size_t i = 0; i < nodeDB->getNumMeshNodes(); i++) {
        meshtastic_NodeInfoLite *n = nodeDB->getMeshNodeByIndex(i);
        if (!n || n->num == nodeDB->getNodeNum() || !n->num)
            continue;
        if ((n->long_name[0] && strcasecmp(n->long_name, name) == 0) ||
            (n->short_name[0] && strcasecmp(n->short_name, name) == 0))
            return n->num;
    }
    return 0;
}

void FamilyTrackerModule::sendFindSound(NodeNum to, bool announce)
{
    // Parent -> node: loud repeating find tone (locate a dropped tracker/base).
    // Runs ~2.5 min; any button press on the target cancels it early. The
    // announcing parent (the command's author) posts one group-chat line;
    // re-firing parents stay silent.
    const meshtastic_NodeInfoLite *n = nodeDB->getMeshNode(to);
    const char *cn = (n && n->long_name[0]) ? n->long_name
                     : (n && n->short_name[0]) ? n->short_name
                                               : "node";
    sendMessageTo(FAMILYTRACKER_MSG_FIND_SOUND, 0, false, false, 0, to);
    if (announce)
        queueTextAlert("FIND SOUND: listen for the tune at %s", cn);
    LOG_WARN("FamilyTracker: FIND SOUND targeted at 0x%08x (%s) announced=%d", (unsigned)to, cn, announce);
}

void FamilyTrackerModule::unknownChildError(const char *name)
{
    LOG_WARN("FamilyTracker: unknown child \"%s\"", name);
    FT_DEBUG("unknown child \"%s\" - command ignored", name);
    playErrorTone();
#if HAS_SCREEN
    if (screen) {
        char banner[64];
        snprintf(banner, sizeof(banner), "No child \"%s\"", name);
        screen->showSimpleBanner(banner, 5000);
    }
#endif
    notifySelfText("No child named \"%s\" - check spelling", name);
}

void FamilyTrackerModule::sendLostChild(NodeNum target)
{
    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p)
        return;

    // v0.3 layout: bytes 2-5 eventId, 6 flags, 7 ageMin, 8-11 eventTs, 12-15 target.
    uint8_t payload[16] = {0};
    payload[0] = FAMILYTRACKER_PROTOCOL_VERSION;
    payload[1] = FAMILYTRACKER_MSG_LOST_CHILD;
    uint32_t eventTs = getValidTime(RTCQuality::RTCQualityDevice, false);
    payload[8] = eventTs & 0xFF;
    payload[9] = (eventTs >> 8) & 0xFF;
    payload[10] = (eventTs >> 16) & 0xFF;
    payload[11] = (eventTs >> 24) & 0xFF;
    payload[12] = target & 0xFF;
    payload[13] = (target >> 8) & 0xFF;
    payload[14] = (target >> 16) & 0xFF;
    payload[15] = (target >> 24) & 0xFF;
    p->to = NODENUM_BROADCAST;
    p->decoded.payload.size = sizeof(payload);
    memcpy(p->decoded.payload.bytes, payload, sizeof(payload));
    service->sendToMesh(p, RX_SRC_LOCAL, true);

    // Regroup: siblings come back when a child is reported lost. Silent datagram
    // only - the "X is lost" announcement IS this event's chat narrative, so an
    // extra "COME BACK:" text would double-report the same thing.
    sendComeBackMsg(NODENUM_BROADCAST, false);

    const meshtastic_NodeInfoLite *c = nodeDB->getMeshNode(target);
    const char *cn = (c && c->long_name[0]) ? c->long_name : (c && c->short_name[0]) ? c->short_name : "Child";
    LOG_WARN("FamilyTracker: LOST CHILD %s (0x%08x) + come-back broadcast", cn, target);
}

// Human-readable alert broadcast on the family channel (TEXT_MESSAGE_APP).
// Serves both as UX (parents see a readable message in the app/console) and as an
// observable side-channel for the module's actions (the text portnum is the
// proven-working RF path).
void FamilyTrackerModule::sendTextAlert(const char *format, ...)
{
    char buf[120];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p)
        return;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->to = NODENUM_BROADCAST;
    p->decoded.payload.size = strnlen(buf, sizeof(buf));
    memcpy(p->decoded.payload.bytes, buf, p->decoded.payload.size);
    service->sendToMesh(p, RX_SRC_LOCAL, true);
    LOG_INFO("FamilyTracker: TEXT alert: %s", buf);
}

void FamilyTrackerModule::queueTextAlert(const char *format, ...)
{
    // Deferred by one runOnce tick: sending TEXT_MESSAGE_APP from inside the RX
    // path loops the packet back into the message store, whose flash save nests
    // spiLock and hangs the T1. Safe from the main thread too (just a member
    // write) so it is used for every RX-path text.
    // BUG-017: 4-slot ring buffer - a single slot dropped texts when two events
    // queued between flushes (e.g. panic text + regroup). Overflow drops the
    // OLDEST entry so the freshest event always survives.
    va_list args;
    va_start(args, format);
    {
        concurrency::LockGuard guard(&stateLock);
        vsnprintf(alertQueue[alertQueueHead], sizeof(alertQueue[0]), format, args);
        alertQueueHead = (alertQueueHead + 1) % ALERT_QUEUE_LEN;
        if (alertQueueCount < ALERT_QUEUE_LEN)
            alertQueueCount++;
        else
            LOG_WARN("FamilyTracker: alert queue overflow (oldest text dropped)");
    }
    va_end(args);
}

#ifdef FAMILY_DEBUG_CHAT
void FamilyTrackerModule::ftDebug(const char *format, ...)
{
    // ENH-009 developer debug mode: echo a protocol event into family chat with
    // a consistent "[FT] " prefix (consumed silently by every node via
    // isFamilyAlertText, so debug lines never trip tones or command matching).
    // Build-time gated (-DFAMILY_DEBUG_CHAT=1); release builds never compile it.
    char buf[96];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    LOG_INFO("FamilyTracker: [FT] %s", buf);
    queueTextAlert("[FT] %s", buf);
}
#endif

bool FamilyTrackerModule::isValidMessage(const meshtastic_MeshPacket &mp, uint8_t *msgType, uint32_t *eventId,
                                         uint32_t *eventTs, uint8_t *flags, uint8_t *ageMin,
                                         meshtastic_PositionLite *pos)
{
    const uint8_t *b = mp.decoded.payload.bytes;
    if (mp.decoded.payload.size < 12)
        return false;
    if (b[0] != FAMILYTRACKER_PROTOCOL_VERSION)
        return false;

    *msgType = b[1];
    *eventId = (uint32_t)b[2] | ((uint32_t)b[3] << 8) | ((uint32_t)b[4] << 16) | ((uint32_t)b[5] << 24);
    *flags = b[6];
    *ageMin = b[7];
    *eventTs = (uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16) | ((uint32_t)b[11] << 24);
    memset(pos, 0, sizeof(*pos));

    if ((*flags & FAMILYTRACKER_FLAG_HAS_POS) && mp.decoded.payload.size >= 22) {
        pos->latitude_i = (int32_t)((uint32_t)b[12] | ((uint32_t)b[13] << 8) | ((uint32_t)b[14] << 16) |
                                    ((uint32_t)b[15] << 24));
        pos->longitude_i = (int32_t)((uint32_t)b[16] | ((uint32_t)b[17] << 8) | ((uint32_t)b[18] << 16) |
                                     ((uint32_t)b[19] << 24));
        pos->altitude = (int16_t)((uint16_t)b[20] | ((uint16_t)b[21] << 8));
    }
    return true;
}

// Case-insensitive substring search (strcasestr is not portable across the
// ESP32/nRF52 newlib builds).
static const char *findNoCase(const char *haystack, const char *needle)
{
    size_t nlen = strlen(needle);
    for (const char *h = haystack; *h; h++) {
        if (strncasecmp(h, needle, nlen) == 0)
            return h;
    }
    return nullptr;
}

ProcessMessage FamilyTrackerModule::handleTextCommand(const meshtastic_MeshPacket &mp)
{
    if (!isParent())
        return ProcessMessage::CONTINUE;

    char buf[64];
    size_t len = mp.decoded.payload.size;
    if (len >= sizeof(buf))
        len = sizeof(buf) - 1;
    memcpy(buf, mp.decoded.payload.bytes, len);
    buf[len] = '\0';

    // Dedup text-command echoes: the same command from the same sender arriving
    // twice within 30s (mesh retransmission) fires the datagram only once.
    static NodeNum lastCmdFrom = 0;
    static uint32_t lastCmdMs = 0;
    static char lastCmd[64] = {0};
    uint32_t nowMs = millis();
    bool isDup = (nowMs - lastCmdMs < 30000) && lastCmdFrom == mp.from &&
                 strncasecmp(lastCmd, buf, sizeof(lastCmd)) == 0;
    lastCmdFrom = mp.from;
    lastCmdMs = nowMs;
    strncpy(lastCmd, buf, sizeof(lastCmd) - 1);
    lastCmd[sizeof(lastCmd) - 1] = '\0';
    if (isDup)
        return ProcessMessage::CONTINUE;

    // Children can't issue parent commands via text (they might accidentally
    // trigger "come back" on a sibling). Their texts are still displayed
    // normally by the normal MESSAGE_TEXT path.
    {
        const meshtastic_NodeInfoLite *sender = nodeDB->getMeshNode(mp.from);
        if (sender && sender->role == meshtastic_Config_DeviceConfig_Role_TRACKER)
            return ProcessMessage::CONTINUE;
    }

    // The loopback copy of our OWN text (Router::sendLocal delivers local
    // broadcasts back to us) is the AUTHOR's firing - it may announce in chat.
    // Copies relayed from OTHER parents re-fire the datagram silently so the
    // group chat isn't spammed with duplicate announcements.
    bool authored = (mp.from == nodeDB->getNodeNum());

    // "found" / "found <name>" (phone text): global stand-down, or clear ONE
    // child. Every parent fires (idempotent); only the author announces.
    if (matchCommand(buf, "found")) {
        char name[32];
        extractName(buf + 5, name, sizeof(name)); 
        if (name[0]) {
            NodeNum target = findChildByName(name);
            if (target) {
                sendFoundTo(target);
                if (!isBase())
                    playFoundMelody(); // parents also hear the success tone
                lostDeclaredByUs.erase(target);
                char still[96];
                if (authored && stillMissingNames(still, sizeof(still)))
                    queueTextAlert("STILL MISSING: %s", still);
                return ProcessMessage::STOP;
            }
            unknownChildError(name);
            return ProcessMessage::STOP;
        }
        sendFound();
        return ProcessMessage::STOP;
    }

    // "come back" / "come back <name>" (phone text)
    if (matchCommand(buf, "come back")) {
        char name[32];
        extractName(buf + 9, name, sizeof(name));
        if (name[0]) {
            NodeNum target = findChildByName(name);
            if (target) {
                if (authored)
                    sendComeBackTo(target); // announces
                else
                    sendComeBackMsg(target, false); // silent datagram only
                return ProcessMessage::STOP;
            }
            if (authored)
                unknownChildError(name);
            return ProcessMessage::STOP;
        }
        if (authored)
            sendComeBack(); // announcing broadcast
        else
            sendComeBackMsg(NODENUM_BROADCAST, false);
        return ProcessMessage::STOP;
    }

    char name[32];
    const char *src = nullptr;

    // "find <name>" (phone text): locate a dropped node by sound
    if (matchCommand(buf, "find")) {
        extractName(buf + 4, name, sizeof(name));
        if (!name[0]) {
            playErrorTone();
#if HAS_SCREEN
            if (screen)
                screen->showSimpleBanner("Usage: find <name>", 5000);
#endif
            return ProcessMessage::STOP;
        }
        NodeNum target = findChildByName(name);
        if (!target)
            target = findAnyNodeByName(name);
        if (target) {
            sendFindSound(target, authored);
            return ProcessMessage::STOP;
        }
        if (authored)
            unknownChildError(name);
        return ProcessMessage::STOP;
    }

    // "on my way" / "coming now" / "calling for help" / "stay put" (phone text):
    // parent acknowledgement to a child in panic.
    {
        int preset = onWayPresetFromText(buf);
        if (preset >= 0) {
            // Extract the child name after the longest matching phrase, or fall
            // back to the most recently panicking child.
            NodeNum target = 0;
            NodeNum lastPanic = 0;
            char nm[32] = {0};
            {
                // Locked snapshot of the RX-thread panic target (B3).
                concurrency::LockGuard guard(&stateLock);
                if (millis() - lastPanicChildMs < 300000)
                    lastPanic = lastPanicChild;
            }
            const char *phrases[] = {"calling for help", "calling help", "on my way",
                                     "coming now", "coming", "stay put", "stay where"};
            for (const char *ph : phrases) {
                const char *pos = findNoCase(buf, ph);
                if (pos) {
                    extractName(pos + strlen(ph), nm, sizeof(nm));
                    break;
                }
            }
            if (nm[0])
                target = findChildByName(nm);
            if (!target && nm[0])
                target = findAnyNodeByName(nm);
            if (!target)
                target = lastPanic;
            if (!target) {
                playErrorTone();
                if (authored) {
                    notifySelfText("Usage: on my way <child name>");
                }
                return ProcessMessage::STOP;
            }
            uint32_t eventId = 0;
            {
                concurrency::LockGuard guard(&stateLock);
                auto it = lastPanicEventIdByNode.find(target);
                if (it != lastPanicEventIdByNode.end())
                    eventId = it->second;
            }
            sendOnWay(eventId, preset, target, authored);
            return ProcessMessage::STOP;
        }
    }

    // "<name> is lost" (app-side canned message)
    const char *p = findNoCase(buf, " is lost");
    if (p && p > buf) {
        size_t n = p - buf;
        if (n >= sizeof(name))
            n = sizeof(name) - 1;
        memcpy(name, buf, n);
        name[n] = '\0';
        src = name;
    } else if (matchCommand(buf, "lost")) {
        // "lost <name>" or bare "lost"
        extractName(buf + 4, name, sizeof(name));
        if (!name[0]) {
            pickLostChild();
            return ProcessMessage::STOP;
        }
        src = name;
    } else {
        return ProcessMessage::CONTINUE;
    }

    NodeNum target = findChildByName(src);
    if (target) {
        if (authored)
            lostDeclaredByUs.insert(target); // only the authoring parent tracks
        sendLostChild(target);
        return ProcessMessage::STOP;
    }
    // Local beep/banner only - the sender resolves their own text via the
    // loopback, so the typo-ing node gives itself the error. No channel spam.
    unknownChildError(src);
    return ProcessMessage::STOP;
}

void FamilyTrackerModule::pickLostChild()
{
#if HAS_SCREEN
    if (!screen)
        return;

    // Populate the picker from the children that have actually checked in
    // (heartbeat), so only live children with real names are offered.
    static std::vector<uint32_t> lostChildList;
    lostChildList.clear();
    {
        concurrency::LockGuard guard(&childSeenLock);
        for (const auto &kv : childLastSeenMs)
            lostChildList.push_back(kv.first);
    }
    if (lostChildList.empty()) {
        queueTextAlert("No children have checked in yet");
        return;
    }

    screen->showNodePicker("Select lost child", 60000, [](uint32_t num) {
        if (familyTrackerModule && num) {
            familyTrackerModule->lostDeclaredByUs.insert(num);
            familyTrackerModule->sendLostChild(num);
        }
    });
    // set after showNodePicker (which resets the filter to "all nodes")
    graphics::NotificationRenderer::setNodePickerFilter(&lostChildList);
#endif
}

// True for texts the family module itself generates (panic announcement, missed
// check-in, found, on-the-way, ...). Consumed in handleReceived so these never
// also trip the generic Meshtastic notification tone (BUG-002/003/004, ARCH §4).
static bool isFamilyAlertText(const meshtastic_MeshPacket &mp)
{
    char buf[120];
    size_t len = mp.decoded.payload.size;
    if (len >= sizeof(buf))
        len = sizeof(buf) - 1;
    memcpy(buf, mp.decoded.payload.bytes, len);
    buf[len] = '\0';

    static const char *const prefixes[] = {
        "MISSED CHECK-IN: ",
        "CHECK-IN RESUMED: ",
        "LOW BATTERY: ",
        "FOUND: ",
        "COME BACK: ",
        "FIND SOUND: ",
        "No child named \"",
        "No children have checked in yet",
        "[FT] ", // ENH-009 developer debug-chat echoes
    };
    for (const char *p : prefixes)
        if (strncasecmp(buf, p, strlen(p)) == 0)
            return true;

    if (findNoCase(buf, "PANIC button pressed at") || findNoCase(buf, " is lost - please help find me"))
        return true;
    // Parent "on the way" responses: "<short>: On my way!" etc.
    for (int i = 0; i < FAMILYTRACKER_ON_WAY_COUNT; i++)
        if (findNoCase(buf, familyTrackerOnWayMessages[i]))
            return true;
    return false;
}

// True for raw family COMMAND texts typed by humans ("found", "come back",
// "lost Alice", "Alice is lost"). These must not trip the generic notification
// tone either (BUG-016): the datagram already fired from the acting parent(s),
// so the bare text is just an echo - consumed like an alert, still forwarded to
// phones for the group chat. Only reached AFTER isFamilyAlertText returned
// false, so generated narratives ("... is lost - please help find me") never
// match here.
static bool isFamilyCommandText(const meshtastic_MeshPacket &mp)
{
    char buf[64];
    size_t len = mp.decoded.payload.size;
    if (len >= sizeof(buf))
        len = sizeof(buf) - 1;
    memcpy(buf, mp.decoded.payload.bytes, len);
    buf[len] = '\0';

    if (matchCommand(buf, "found") || matchCommand(buf, "come back") || matchCommand(buf, "lost") ||
        matchCommand(buf, "find"))
        return true;
    if (findNoCase(buf, " is lost"))
        return true;
    return false;
}

ProcessMessage FamilyTrackerModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // BUG-001: ANY packet authored by a tracked child is proof of contact -
    // periodic positions, telemetry, texts, family datagrams alike. The
    // missed-check-in watchdog may only fire after 10 min with NO traffic
    // whatsoever, so contact tracking can't depend on our own CHECKIN alone.
    if (isParent() && mp.from != nodeDB->getNodeNum()) {
        const meshtastic_NodeInfoLite *snd = nodeDB->getMeshNode(mp.from);
        if (snd && snd->role == meshtastic_Config_DeviceConfig_Role_TRACKER)
            markChildSeen(mp.from, millis());
    }

    // Parent-contact tracking (child watchdog mirror): any text authored by a
    // non-tracker proves a parent is reachable.
    if (mp.from != nodeDB->getNodeNum()) {
        const meshtastic_NodeInfoLite *txr = nodeDB->getMeshNode(mp.from);
        if (txr && txr->role != meshtastic_Config_DeviceConfig_Role_TRACKER &&
            txr->role != meshtastic_Config_DeviceConfig_Role_TAK_TRACKER)
            markParentHeard();
    }

    // Human-readable family commands ("Child 1 is lost") arrive as text ON THE
    // CHANNEL. Direct messages (to == us) must flow untouched on every role
    // (REQ-001): never command-parsed, never tone-suppressed - a child's phone
    // chatting with a parent is plain Meshtastic messaging.
    if (mp.decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP) {
        if (!isBroadcast(mp.to))
            return ProcessMessage::CONTINUE;

        // Consume OUR OWN alert texts first (BUG-004/009): the child's
        // "Alice is lost - please help find me" contains " is lost", so it must
        // never re-enter command matching (that would loop LOST_CHILD -> text ->
        // LOST_CHILD...). This also suppresses the generic Meshtastic tone for
        // every family alert (BUG-002/003/004, ARCH §4) while the message still
        // displays (TextMessageModule already stored it).
        if (isFamilyAlertText(mp)) {
#ifdef FAMILY_TEST_HOOKS
            // Test builds: surface family alert texts to the phone/serial API so
            // the automated harness can see FOUND/check-in texts, but still skip
            // command re-entry (BUG-004 loop protection). Buzzer is OFF under
            // FAMILY_TEST_HOOKS, so the suppressed generic tone is moot.
            return ProcessMessage::CONTINUE;
#else
            // Production: forward the alert to the phone/serial API so it shows
            // in the family group chat (same copy+queue path RoutingModule uses
            // for every received packet), then STOP so the generic Meshtastic
            // notification tone never also fires (BUG-002/003/004) and the text
            // can't re-enter command matching.
            if (auto *toPhone = packetPool.allocCopy(mp))
                service->sendToPhone(toPhone);
            return ProcessMessage::STOP;
#endif
        }
        ProcessMessage r = handleTextCommand(mp);
        if (r == ProcessMessage::STOP)
            return r;
        // Raw command echoes ("found", "come back Alice", ...): the acting
        // parents already fired the datagrams above; consume the bare text so
        // children don't ALSO play the generic notification tone over the
        // family tone (BUG-016). Still forwarded to phones for group chat.
        if (isFamilyCommandText(mp)) {
#ifdef FAMILY_TEST_HOOKS
            return ProcessMessage::CONTINUE;
#else
            if (auto *toPhone = packetPool.allocCopy(mp))
                service->sendToPhone(toPhone);
            return ProcessMessage::STOP;
#endif
        }
        return ProcessMessage::CONTINUE;
    }

    uint8_t msgType, flags, ageMin;
    uint32_t eventId, eventTs;
    meshtastic_PositionLite pos;
    if (!isValidMessage(mp, &msgType, &eventId, &eventTs, &flags, &ageMin, &pos))
        return ProcessMessage::CONTINUE;

    // Family datagram from a parent = parent-contact proof for the child watchdog.
    if (mp.from != nodeDB->getNodeNum()) {
        const meshtastic_NodeInfoLite *txr = nodeDB->getMeshNode(mp.from);
        if (txr && txr->role != meshtastic_Config_DeviceConfig_Role_TRACKER &&
            txr->role != meshtastic_Config_DeviceConfig_Role_TAK_TRACKER)
            markParentHeard();
    }

    // A valid v0.3 family datagram proves the sender is family (private channel
    // + protocol check): auto-favourite it once for the Friend-Finder/nearest-
    // parent UI. Deferred to runOnce - set_favorite() writes node DB to flash.
    autoFavourite(mp.from);

    switch (msgType) {
    case FAMILYTRACKER_MSG_PANIC: {
        // A child panicked. Parents alert + ACK; children ignore.
        if (isParent()) {
            markChildSeen(mp.from, millis()); // panic is also proof-of-life (SPEC §18)
            // Dedup by (from, eventId) within a window (BUG-003/004/008): the
            // SAME event retransmitted by the mesh must not re-alert, but a
            // FRESH event (retrigger) always does.
            uint32_t nowMs = millis();
            auto it = lastPanicEventIdByNode.find(mp.from);
            bool alreadyAlerted = it != lastPanicEventIdByNode.end() && it->second == eventId &&
                                  (nowMs - lastPanicAlertMsByNode[mp.from] < FAMILYTRACKER_PANIC_DEDUP_MS);
            if (!alreadyAlerted) {
                lastPanicEventIdByNode[mp.from] = eventId;
                lastPanicAlertMsByNode[mp.from] = nowMs;
                // Queue the whole alert (banner + buzzer + ACK) for runOnce:
                // doing it here inside the radio RX handler contends with the
                // radio/flash SPI lock on the nRF52 and intermittently hangs.
                // Ring slot (not single): two children panicking within one
                // tick must BOTH alarm.
                {
                    concurrency::LockGuard guard(&stateLock);
                    PendingPanic &slot = pendingPanics[pendingPanicHead];
                    slot.active = true;
                    slot.from = mp.from;
                    slot.eventId = eventId;
                    slot.eventTs = eventTs;
                    slot.pos = pos;
                    slot.stale = (flags & FAMILYTRACKER_FLAG_POS_STALE) != 0;
                    slot.ageMin = ageMin;
                    slot.hasPos = (flags & FAMILYTRACKER_FLAG_HAS_POS) != 0;
                    pendingPanicHead = (pendingPanicHead + 1) % PENDING_PANIC_LEN;
                    if (pendingPanicCount < PENDING_PANIC_LEN)
                        pendingPanicCount++;
                    else
                        LOG_WARN("FamilyTracker: panic queue overflow (oldest dropped)");
                    // ENH-001: persistent parent awareness until stood down.
                    parentAlertNodes.insert(mp.from);
                    lastPanicChild = mp.from; // target for a quick unnamed "on my way"
                    lastPanicChildMs = nowMs;
                }
                FT_DEBUG("panic rx event=%u from=0x%04x pos=%s%s", (unsigned)eventId, (unsigned)mp.from,
                         (flags & FAMILYTRACKER_FLAG_HAS_POS) ? ((flags & FAMILYTRACKER_FLAG_POS_STALE) ? "stale" : "fresh")
                                                              : "none",
                         alreadyAlerted ? " dup" : "");
            } else {
                FT_DEBUG("panic rx event=%u from=0x%04x DEDUPED", (unsigned)eventId, (unsigned)mp.from);
            }
        }
        break;
    }

    case FAMILYTRACKER_MSG_PANIC_ACK:
        // Our panic was acknowledged: ACKED tone (SPEC §34)
        // Multi-Child (§18A): ACKs are now targeted at the specific child (mp.to
        // == us), so eventId alone can no longer collide with another child's
        // locally-numbered event. Accept targeted ACKs, and broadcast ACKs only
        // when they match our own outstanding panic (backwards compat).
        if (isChild() && eventId == lastPanicEventId && (mp.to == nodeDB->getNodeNum() || isBroadcast(mp.to))) {
            playPanicResponse(); // rising "response" completes the call/response (SPEC §34)
            FT_DEBUG("panic acked event=%u by=0x%04x", (unsigned)eventId, (unsigned)mp.from);
            LOG_INFO("FamilyTracker: PANIC ACKED event=%u from 0x%08x", eventId, mp.from);
        }
        break;

    case FAMILYTRACKER_MSG_PARENT_ON_WAY:
        // Parent confirmed they read the panic and are on the way (SPEC §34A).
        // Child: distinct "on the way" tone + short on-screen message.
        if (isChild()) {
            uint8_t presetIndex = ageMin; // carried in the ageMin byte
            if (presetIndex >= FAMILYTRACKER_ON_WAY_COUNT)
                presetIndex = 0;
            playStartMelody(); // distinct from ACKED boop (SPEC §34A)
            FT_DEBUG("on the way rx preset=%u from=0x%04x", presetIndex, (unsigned)mp.from);
            if (screen) {
                char banner[80];
                snprintf(banner, sizeof(banner), "Parent on the way:\n%s", familyTrackerOnWayMessages[presetIndex]);
                screen->showSimpleBanner(banner, 8000);
            }
            LOG_INFO("FamilyTracker: PARENT ON THE WAY from 0x%08x: '%s'", mp.from,
                     familyTrackerOnWayMessages[presetIndex]);
        }
        break;

#ifdef FAMILY_TEST_HOOKS
    case FAMILYTRACKER_MSG_ON_WAY_TRIGGER:
        // Remote trigger for hands-off testing (like PANIC_TRIGGER): a parent
        // responds "on the way" to the most recent panicking child. presetIndex
        // rides in the ageMin byte; eventId selects the panic event.
        if (isParent()) {
            uint8_t presetIndex = ageMin;
            if (presetIndex >= FAMILYTRACKER_ON_WAY_COUNT)
                presetIndex = 0;
            LOG_INFO("FamilyTracker: ON_WAY_TRIGGER from 0x%08x preset=%u -> respond", mp.from, presetIndex);
            sendOnWay(eventId, presetIndex, mp.from);
        }
        break;
#endif

    case FAMILYTRACKER_MSG_LOCATE_REQ:
        // Parent requests our position (SPEC §25/§26)
        if (isChild()) {
            buzzerBeep(false); // locate tone (SPEC §27)
            LOG_INFO("FamilyTracker: LOCATE_REQ from 0x%08x -> responding", mp.from);
            sendMessage(FAMILYTRACKER_MSG_LOCATE_RESP, eventId, true, false, 0); // immediate latest (SPEC §26.2)
            triggerFreshFix();                                                    // then try fresh (SPEC §26.3)
        }
        break;

#ifdef FAMILY_TEST_HOOKS
    case FAMILYTRACKER_MSG_PANIC_TRIGGER:
        // Remote panic trigger (family-only): makes a child run the SAME panic send
        // path as the physical button. Testable over USB/LoRa without pressing the
        // button (SPEC §32); also enables parent-initiated drills.
        if (isChild()) {
            LOG_INFO("FamilyTracker: PANIC_TRIGGER from 0x%08x -> panic", mp.from);
            sendPanic();
        }
        break;
#endif

    case FAMILYTRACKER_MSG_LOCATE_RESP:
        // Child's locate response (with optional fresh-fix update)
        if (isParent())
            markChildSeen(mp.from, millis()); // locate response is also proof-of-life (SPEC §18)
        LOG_INFO("FamilyTracker: LOCATE_RESP from 0x%08x lat=%ld lon=%ld%s", mp.from, (long)pos.latitude_i,
                 (long)pos.longitude_i, (flags & FAMILYTRACKER_FLAG_POS_STALE) ? " (stale)" : "");
        break;

    case FAMILYTRACKER_MSG_CHECKIN:
        // Child heartbeat (parent): confirms alive + carries position/battery.
        // This is the authoritative "child is alive" pulse (SPEC §18): track it
        // directly (PRIVATE_APP doesn't update nodedb lastHeard).
        if (isParent())
            markChildSeen(mp.from, millis());
        // No per-checkin [FT] chat line - periodic heartbeats would flood the
        // group chat and drown real messages (LOG only).
        LOG_INFO("FamilyTracker: CHECKIN from 0x%08x%s%s", mp.from,
                 (flags & FAMILYTRACKER_FLAG_HAS_POS) ? " pos" : " (no pos)",
                 (flags & FAMILYTRACKER_FLAG_POS_STALE) ? " (stale)" : "");
        break;

#ifdef FAMILY_TEST_HOOKS
    case FAMILYTRACKER_MSG_CONFIG: {
        // Remote test-config: set the parent watchdog timeout + low-battery
        // threshold so §18/§20/§23 can be exercised without a 10-min wait.
        // v0.3 layout: payload byte 7 = timeout in seconds (0 = leave
        // unchanged), byte 12 = low-battery percent (0 = leave unchanged).
        if (isParent() && mp.decoded.payload.size >= 13) {
            uint8_t t = mp.decoded.payload.bytes[7];
            uint8_t b = mp.decoded.payload.bytes[12];
            if (t > 0) {
                missedTimeoutSecs = max((uint32_t)30, (uint32_t)t);
                LOG_WARN("FamilyTracker: CONFIG timeout_s -> %u", missedTimeoutSecs);
            }
            if (b > 0) {
                lowBatteryPct = max((uint8_t)1, b);
                LOG_WARN("FamilyTracker: CONFIG low_battery_pct -> %u", lowBatteryPct);
            }
        }
        break;
    }
#endif

    case FAMILYTRACKER_MSG_COME_BACK:
        // Parent -> child (or sibling child -> group, BUG-009): "return to
        // parent". Distinct tone + on-screen message. A child ignores its own
        // RETURN_TO_PARENT loopback so the panicking child isn't double-tuned.
        if (mp.from != nodeDB->getNodeNum() && isChild() && (mp.to == nodeDB->getNodeNum() || isBroadcast(mp.to))) {
            // Both parents may fire on the same command text - play the tune
            // once per burst (10s guard) instead of twice.
            if (millis() - lastComeBackRxMs >= 10000UL) {
                lastComeBackRxMs = millis();
                playMarioMelody(); // distinct "come back" tune (SPEC §34A)
                FT_DEBUG("come back rx from=0x%04x%s", (unsigned)mp.from, isBroadcast(mp.to) ? " (group)" : "");
            }
            if (screen)
                screen->showSimpleBanner("Come back now!", 8000);
            LOG_INFO("FamilyTracker: COME BACK from 0x%08x", mp.from);
        }
        break;

    case FAMILYTRACKER_MSG_LOST_CHILD: {
        // Parent -> group: "child X is lost". Target node rides in payload
        // bytes 12-15 (uint32 LE, v0.3 layout). The target child enters lost
        // mode (fast check-in) and announces itself; parents raise a local alert.
        uint32_t target = 0;
        if (mp.decoded.payload.size >= 16) {
            const uint8_t *b = mp.decoded.payload.bytes;
            target = (uint32_t)b[12] | ((uint32_t)b[13] << 8) | ((uint32_t)b[14] << 16) | ((uint32_t)b[15] << 24);
        }
        if (isChild() && target == nodeDB->getNodeNum()) {
            bool alreadyLost = lostModeActive;
            lostModeActive = true; // persists until a parent sends FOUND
            if (!alreadyLost) {
                // Use the same come-back tune as COME_BACK with the 10s guard
                // so that both parents sending LOST_CHILD simultaneously only
                // produces one play-through (same signal the child already knows).
                if (millis() - lastComeBackRxMs >= 10000UL) {
                    lastComeBackRxMs = millis();
                    playMarioMelody();
                    FT_DEBUG("lost mode ON (declared by 0x%04x)", (unsigned)mp.from);
                }
                // Announce ONCE per lost episode - re-declarations must not
                // re-spam the group chat with "please help find me".
                if (screen)
                    screen->showSimpleBanner("Lost - come back now!", 15000);
                char lost[64];
                snprintf(lost, sizeof(lost), "%s is lost - please help find me", owner.short_name);
                queueTextAlert("%s", lost); // deferred: RX-path text would re-enter the message store
                LOG_WARN("FamilyTracker: LOST CHILD (me) - entering lost mode");
            } else if (screen) {
                screen->showSimpleBanner("Lost - come back now!", 15000);
            }
        } else if (isParent()) {
            const meshtastic_NodeInfoLite *c = nodeDB->getMeshNode(target);
            const char *cn = "Child";
            if (c && c->long_name[0])
                cn = c->long_name;
            else if (c && c->short_name[0])
                cn = c->short_name;
            playLostAlert(); // distinct from the child's SOS
            FT_DEBUG("lost child alert target=0x%04x", (unsigned)target);
            if (screen) {
                char banner[80];
                snprintf(banner, sizeof(banner), "%s is lost - look for them", cn);
                screen->showSimpleBanner(banner, 15000);
            }
            LOG_WARN("FamilyTracker: LOST CHILD %s (0x%08x) - please look for them", cn, target);
        }
        break;
    }

    case FAMILYTRACKER_MSG_FIND_SOUND:
        // Any role: loud repeating find tone (locate a node in a crowd / dropped
        // in grass). runOnce() re-beeps it and stops after the timeout.
        if (mp.to == nodeDB->getNodeNum() || isBroadcast(mp.to)) {
            findSoundUntilMs = millis() + FAMILYTRACKER_FIND_SOUND_SECS * 1000UL;
            lastFindBeepMs = 0; // re-beep immediately on the next fast tick
            LOG_INFO("FamilyTracker: FIND SOUND from 0x%08x", mp.from);
        }
        break;

    case FAMILYTRACKER_MSG_CANCEL:
        // Stand-down: a parent cancels an active alert (panic / lost-mode /
        // find-sound) on this node. Only a non-child (parent) may send it.
        {
            const meshtastic_NodeInfoLite *sender = nodeDB->getMeshNode(mp.from);
            bool senderIsParent =
                sender && sender->role != meshtastic_Config_DeviceConfig_Role_TRACKER &&
                sender->role != meshtastic_Config_DeviceConfig_Role_TAK_TRACKER;
            if (!senderIsParent) {
                LOG_WARN("FamilyTracker: CANCEL ignored (sender 0x%08x is not a parent)", mp.from);
                break;
            }
        }
        if (mp.to == nodeDB->getNodeNum() || isBroadcast(mp.to)) {
            const meshtastic_NodeInfoLite *sender = nodeDB->getMeshNode(mp.from);
            const char *pn = (sender && sender->long_name[0])
                                 ? sender->long_name
                                 : (sender && sender->short_name[0]) ? sender->short_name : "Parent";
            // ENH-001 bookkeeping: a broadcast stand-down clears every parent-
            // side panic indication; a targeted one clears just that child.
            {
                concurrency::LockGuard guard(&stateLock);
                if (isBroadcast(mp.to))
                    parentAlertNodes.clear();
                else
                    parentAlertNodes.erase(mp.to);
            }
            // Only the node with an ACTUALLY active alert announces "found"
            // (so a broadcast stand-down doesn't make every node claim it was
            // found). Explicit state machine - NOT lastPanicEventId, which a
            // child never sets for someone else's panic (BUG: siblings used to
            // announce FOUND for panics they only overheard).
            bool wasActive = findSoundUntilMs != 0 || lostModeActive ||
                             (isChild() && panicState == FamilyPanicState::PANIC_ACTIVE);
            findSoundUntilMs = 0;
            lostModeActive = false;
            lastPanicEventId = 0; // stand down any outstanding panic
            // Explicit panic state transition PANIC_ACTIVE -> PANIC_CLEARED
            // (ARCH §2, BUG-012/013). Cooldown reset so the child can trigger a
            // fresh panic immediately after being found.
            if (isChild() && panicState == FamilyPanicState::PANIC_ACTIVE) {
                panicState = FamilyPanicState::PANIC_CLEARED;
                lastPanicSentMs = 0;
                FT_DEBUG("panic state -> CLEARED (by 0x%04x)", (unsigned)mp.from);
                LOG_WARN("FamilyTracker: panic state PANIC_ACTIVE -> PANIC_CLEARED");
            }
            if (!isBase())
                playFoundMelody(); // "level complete" success tone
            FT_DEBUG("found rx by=0x%04x wasActive=%d", (unsigned)mp.from, wasActive);
            if (screen)
                screen->showSimpleBanner("Found - standing down", 8000);
            if (wasActive)
                queueTextAlert("FOUND: %s - standing down", owner.short_name);
            LOG_WARN("FamilyTracker: FOUND by %s (0x%08x)", pn, mp.from);
        }
        break;

    case FAMILYTRACKER_MSG_MISSED_ALERT: {
        // Another parent already reported this child missing to the group.
        // Start our own suppression window so N parents don't spam N identical
        // "MISSED CHECK-IN" texts (local tone still plays on our own detection).
        uint32_t target = 0;
        if (mp.decoded.payload.size >= 16) {
            const uint8_t *b = mp.decoded.payload.bytes;
            target = (uint32_t)b[12] | ((uint32_t)b[13] << 8) | ((uint32_t)b[14] << 16) | ((uint32_t)b[15] << 24);
        }
        if (isParent() && target && mp.from != nodeDB->getNodeNum()) {
            concurrency::LockGuard guard(&stateLock);
            missedSuppressedUntilMs[target] = millis() + FAMILYTRACKER_MISSED_SUPPRESS_MS;
            LOG_INFO("FamilyTracker: MISSED_ALERT child=0x%08x from 0x%08x - suppressing own group report", target,
                     mp.from);
        }
        break;
    }

    case FAMILYTRACKER_MSG_PARENT_PRESENCE:
        // Parent startup presence + position (BUG-011/ENH-007). Children use it
        // to seed/refresh the nearest-parent display promptly after boot. The
        // child's nodedb also learns the parent via its standard position
        // broadcasts, so this is informational + a warm-up only.
        if (isChild()) {
            LOG_INFO("FamilyTracker: PARENT PRESENCE from 0x%08x", mp.from);
            // updateNearestParentDisplay() self-gates on GPS availability.
            updateNearestParentDisplay();
        }
        break;

    default:
        return ProcessMessage::CONTINUE;
    }
#ifdef FAMILY_TEST_HOOKS
    // Test builds: forward family PRIVATE_APP frames to the phone/serial API so
    // the automated harness can observe CHECKIN/PANIC/LOCATE_RESP/ON_WAY as they
    // flow through the parent. Production builds consume them (STOP) to keep the
    // family protocol private from the generic phone notification path.
    return ProcessMessage::CONTINUE;
#else
    return ProcessMessage::STOP;
#endif
}

// Stationary base mobility guard: a CLIENT_BASE node left at camp / in the car
// stays a silent sentinel. If its GPS shows it moved FAMILYTRACKER_BASE_MOVE_
// METRES from where it first got a fix, an adult has picked it up to join the
// search - promote it to a full CLIENT parent: persisted role change, NodeInfo
// rebroadcast so every child's nodedb (and nearest-parent arrow) updates.
void FamilyTrackerModule::updateBaseMobility()
{
    if (mobilePromoted || config.device.role != meshtastic_Config_DeviceConfig_Role_CLIENT_BASE)
        return;

    const meshtastic_NodeInfoLite *self = nodeDB->getMeshNode(nodeDB->getNodeNum());
    if (!self || !nodeDB->hasValidPosition(self))
        return; // no fix yet - keep waiting for the anchor

    if (!baseAnchorSet) {
        baseAnchorSet = true;
        baseAnchorLat = localPosition.latitude_i;
        baseAnchorLon = localPosition.longitude_i;
        LOG_INFO("FamilyTracker: base anchor set (%.5f, %.5f)", baseAnchorLat * 1e-7, baseAnchorLon * 1e-7);
        return;
    }

    float d = GeoCoord::latLongToMeter(baseAnchorLat * 1e-7, baseAnchorLon * 1e-7, localPosition.latitude_i * 1e-7,
                                       localPosition.longitude_i * 1e-7);
    if (d < FAMILYTRACKER_BASE_MOVE_METRES)
        return;

    // Picked up: become a full parent.
    mobilePromoted = true;
    config.device.role = meshtastic_Config_DeviceConfig_Role_CLIENT;
#ifdef USERPREFS_CONFIG_GPS_UPDATE_INTERVAL
    // Undo the CLIENT_BASE 15-min GPS economy - a moving searcher wants fresh
    // fixes (NodeDB::initConfigIntervals bakes this down to 900 s for base).
    config.position.gps_update_interval = USERPREFS_CONFIG_GPS_UPDATE_INTERVAL;
#endif
    nodeDB->saveToDisk();
    if (nodeInfoModule)
        nodeInfoModule->sendOurNodeInfo(); // children re-role us in their nodedb
    playStartMelody();                     // audible "I'm joining" cue for whoever grabbed it
    LOG_WARN("FamilyTracker: base moved %.0f m - promoted to CLIENT parent", d);
}

// Family remote-admin default: copy each PARENT-role family node's public key
// into our security.admin_key[0..2] so any parent can PKI-admin any family node
// (children included) without manual key exchange. Children run this too - they
// TRUST parents, which is exactly what lets parents admin them. Runs once at
// startup on our own OSThread (flash save), never in the RX path.
void FamilyTrackerModule::syncAdminTrust()
{
    size_t added = 0;
    size_t count = nodeDB->getNumMeshNodes();
    for (size_t i = 0; i < count && added < 3; i++) {
        const meshtastic_NodeInfoLite *n = nodeDB->getMeshNodeByIndex(i);
        if (!n || n->num == nodeDB->getNodeNum())
            continue;
        // Only parent-ish roles are trusted admins (never trackers).
        bool isParentRole =
            IS_ONE_OF(n->role, meshtastic_Config_DeviceConfig_Role_CLIENT, meshtastic_Config_DeviceConfig_Role_CLIENT_MUTE,
                      meshtastic_Config_DeviceConfig_Role_ROUTER, meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT,
                      meshtastic_Config_DeviceConfig_Role_CLIENT_BASE, meshtastic_Config_DeviceConfig_Role_ROUTER_LATE);
        if (!isParentRole)
            continue;
        if (n->public_key.size != 32)
            continue; // no PKI key exchanged yet

        bool alreadyTrusted = false;
        int freeSlot = -1;
        for (int s = 0; s < 3; s++) {
            if (config.security.admin_key[s].size == 32 &&
                memcmp(config.security.admin_key[s].bytes, n->public_key.bytes, 32) == 0) {
                alreadyTrusted = true;
                break;
            }
            if (config.security.admin_key[s].size == 0 && freeSlot < 0)
                freeSlot = s;
        }
        if (alreadyTrusted || freeSlot < 0)
            continue;
        memcpy(config.security.admin_key[freeSlot].bytes, n->public_key.bytes, 32);
        config.security.admin_key[freeSlot].size = 32;
        added++;
        LOG_WARN("FamilyTracker: trusted admin key from family node 0x%08x", (unsigned)n->num);
    }
    if (added > 0) {
        nodeDB->saveToDisk(); // persist so trust survives reboots
        LOG_WARN("FamilyTracker: %u family admin keys saved", (unsigned)added);
    }
}

int FamilyTrackerModule::handleInputEvent(const InputEvent *event)
{
    if (event) {
        switch (event->inputEvent) {
        case INPUT_BROKER_USER_PRESS:
        case INPUT_BROKER_SEND_PING:
        case INPUT_BROKER_ALT_PRESS:
            // A button press while the find sound is active cancels it (and does
            // NOT raise a panic) - so a child/parent can silence "find me" once
            // found without accidentally triggering a panic.
            if (findSoundUntilMs) {
                findSoundUntilMs = 0;
                LOG_INFO("FamilyTracker: FIND SOUND cancelled by button");
                break;
            }
            // BUG-007: while a menu/picker/keyboard overlay is up, presses drive
            // navigation only - never panic.
#if HAS_SCREEN
            if (screen && graphics::NotificationRenderer::isMenuShowing()) {
                LOG_INFO("FamilyTracker: press ignored (menu active)");
                break;
            }
#endif
            if (isChild())
                sendPanic();
            break;
        default:
            break;
        }
    }
    return 0;
}

// Parent watchdog: missed check-in + low battery (SPEC §18-§23).
// Child: periodic CHECKIN heartbeat independent of GPS (SPEC §13/§14/§18).
// Shared SOS-LED driver (ENH-008 child / ENH-001 parent). Phase comes from
// millis() alone, so the pattern keeps running whatever the display does.
// Pass on=false on every tick once alarming ends - the transition releases
// the LED exactly once.
void FamilyTrackerModule::indicateSosLed(bool on)
{
#if defined(PIN_LED1)
    static const uint16_t sosSeq[][2] = {// {ledOn, duration ms} - . . . - - - . . .  (x2 timing for visibility)
                                         {1, 200}, {0, 200}, {1, 200}, {0, 200}, {1, 200}, {0, 200},
                                         {1, 600}, {0, 200}, {1, 600}, {0, 200}, {1, 600}, {0, 200},
                                         {1, 200}, {0, 200}, {1, 200}, {0, 200}, {1, 200}, {0, 1400}};
    const uint16_t sosTotalMs = 6200;
    pinMode(PIN_LED1, OUTPUT);
    if (!on) {
        if (ledSosActive) {
            ledSosActive = false;
            digitalWrite(PIN_LED1, !LED_STATE_ON);
        }
        return;
    }
    ledSosActive = true;
    uint32_t phase = millis() % sosTotalMs;
    uint16_t elapsed = 0;
    for (auto &seg : sosSeq) {
        elapsed += seg[1];
        if (phase < elapsed) {
            digitalWrite(PIN_LED1, seg[0] ? LED_STATE_ON : !LED_STATE_ON);
            break;
        }
    }
#endif
}

int32_t FamilyTrackerModule::runOnce()
{
#ifdef FAMILY_GPS_DIAG
    // ENH-002 temporary T1 hardware diagnostic: hold the GPS receiver fully
    // powered (no duty-cycling between fixes) while the GPS power-management
    // issue is investigated. Broadcast cadence is UNCHANGED, so this affects
    // only power/sampling - never mesh traffic. Kept separate from
    // FAMILY_DEBUG_CHAT because it may stay enabled for weeks.
    if (gps)
        gps->setPowerState(GPSPowerState::GPS_ACTIVE);
#endif

    // Act on a queued panic OUTSIDE the radio RX path: banner + buzzer + ACK.
    // None of these side-effects run inside handleReceived(), which is what
    // hangs the T1 parent.
    for (;;) {
        PendingPanic pp; // copy-out under lock, act on it lock-free
        {
            concurrency::LockGuard guard(&stateLock);
            if (pendingPanicCount == 0)
                break;
            uint8_t tail = (pendingPanicHead + PENDING_PANIC_LEN - pendingPanicCount) % PENDING_PANIC_LEN;
            pp = pendingPanics[tail];
            pendingPanics[tail].active = false;
            pendingPanicCount--;
        }
        renderPanicAlert(pp.from, pp.eventId, pp.eventTs, pp.pos, pp.stale, pp.ageMin, pp.hasPos);
        playPanicAlert(); // alert (distinct from the child's call)
        FT_DEBUG("parent panic alarm event=%u from=0x%04x", (unsigned)pp.eventId, (unsigned)pp.from);
        sendAck(pp.eventId, pp.from); // SPEC §34 — targeted ACK (multi-Child §18A)
        return 200;
    }

    // Flush deferred family texts (queued by queueTextAlert from RX-path code
    // to avoid a reentrant message-store write that hangs the T1 parent).
    for (;;) {
        char buf[sizeof(alertQueue[0])];
        {
            concurrency::LockGuard guard(&stateLock);
            if (alertQueueCount == 0)
                break;
            uint8_t tail = (alertQueueHead + ALERT_QUEUE_LEN - alertQueueCount) % ALERT_QUEUE_LEN;
            memcpy(buf, alertQueue[tail], sizeof(buf));
            alertQueue[tail][0] = '\0';
            alertQueueCount--;
        }
        sendTextAlert("%s", buf);
    }

    // Family remote-admin trust sync: retried over the first ~10 min after boot
    // (peers announce + exchange keys gradually); each retry only touches flash
    // if new keys were actually merged. Cheap nodedb scan otherwise.
    if (trustSyncAttempts < 10 && (lastTrustSyncMs == 0 || millis() - lastTrustSyncMs > 60000UL)) {
        trustSyncAttempts++;
        lastTrustSyncMs = millis();
        syncAdminTrust();
    }

    // One-shot startup announce (BUG-011): broadcast a standard NodeInfo so
    // phones/node-lists show a fresh "last updated" immediately, alongside the
    // family-level CHECKIN/PARENT_PRESENCE sync below.
    if (!startupNodeInfoSent) {
        startupNodeInfoSent = true;
        if (nodeInfoModule) {
            nodeInfoModule->sendOurNodeInfo();
            FT_DEBUG("startup nodeinfo sent (%s)", isChild() ? "child" : isBase() ? "base" : "parent");
        }
    }

    // Fast position follow-up (ENH): once a fresh GPS fix lands within 2 min of
    // boot, push an immediate position-bearing update so the family gets the
    // location quickly instead of waiting a full broadcast interval.
    if (quickFixPending) {
        if (bootAtMs == 0) {
            bootAtMs = millis();
        } else if (millis() - bootAtMs > 120000UL) {
            quickFixPending = false; // window elapsed with no fix; normal cadence continues
        } else {
            meshtastic_PositionLite p;
            bool hp = false, st = false;
            uint8_t am = 0;
            fillBestPosition(&p, &hp, &st, &am);
            if (hp && !st) {
                quickFixPending = false;
                if (isChild())
                    sendMessage(FAMILYTRACKER_MSG_CHECKIN, 0, true, false, 0);
                else
                    sendMessage(FAMILYTRACKER_MSG_PARENT_PRESENCE, 0, true, false, 0);
                FT_DEBUG("gps quickfix sent (%u s after boot)", (unsigned)((millis() - bootAtMs) / 1000));
                LOG_INFO("FamilyTracker: GPS quickfix after %u s", (unsigned)((millis() - bootAtMs) / 1000));
            }
        }
    }

    // (Child/parent persistent alarm indication moved into the role branches
    // below - it must never preempt the periodic check-in, which is exactly
    // how BUG-001's false "missed/resumed" pair was born.)

    // Auto-favourite family nodes (Friend-Finder/ENH): set_favorite() writes the
    // node DB to flash, deferred here out of the RX path. Idempotent - a node
    // already favourited is a no-op.
    {
        concurrency::LockGuard guard(&favouriteLock);
        for (NodeNum num : pendingFavourites)
            nodeDB->set_favorite(true, num);
        pendingFavourites.clear();
    }

    // ENH-001: PARENT persistent panic indication - a child panicked and nobody
    // has stood it down yet. SOS LED (wall-clock phase: keeps flashing while
    // the screen sleeps) + repeating "PANIC: <name>" banner on displays.
#if defined(PIN_LED1)
    indicateSosLed(true); // releases itself when the set is empty
#endif
    bool parentAlarming;
    {
        concurrency::LockGuard guard(&stateLock);
        parentAlarming = !parentAlertNodes.empty();
    }
    if (parentAlarming) {
#if HAS_SCREEN
        NodeNum firstAlert = 0;
        {
            concurrency::LockGuard guard(&stateLock);
            firstAlert = *parentAlertNodes.begin();
        }
        if (screen && millis() - lastAlarmBannerMs >= 8000UL) {
            lastAlarmBannerMs = millis();
            const meshtastic_NodeInfoLite *c = nodeDB->getMeshNode(firstAlert);
            const char *cn = (c && c->long_name[0]) ? c->long_name : (c && c->short_name[0]) ? c->short_name : "child";
            char banner[64];
            snprintf(banner, sizeof(banner), "PANIC: %s", cn);
            screen->showSimpleBanner(banner, 8000);
        }
#endif
        return 100; // fast tick while indicating
    }

    // Find sound: re-beep the loud tone while active (any role). Fast tick so
    // the cadence is responsive; auto-stops when the timeout expires.
    if (findSoundUntilMs) {
        if ((int32_t)(millis() - findSoundUntilMs) >= 0) {
            findSoundUntilMs = 0;
        } else if (millis() - lastFindBeepMs >= FAMILYTRACKER_FIND_BEEP_INTERVAL_MS) {
            lastFindBeepMs = millis();
            playComboTune();
        }
        return 500;
    }

    if (isChild()) {
        // Lost mode: much faster check-in cadence while a search is active.
        // Persists until a parent sends FOUND (no auto-timeout) - the searchers
        // keep tracking until the child is explicitly found.
        uint32_t intervalMs = lostModeActive ? FAMILYTRACKER_LOST_CHECKIN_SECS * 1000UL
                                             : FAMILYTRACKER_CHECKIN_INTERVAL_SECS * 1000UL;
        // Periodic check-in so the parent can monitor us even without a GPS fix.
        // Use millis() (monotonic) not getValidTime() so it works without an RTC.
        // Startup sync (BUG-011/ENH-007): CHECKIN immediately on the first tick
        // so a rebooted child is re-established without waiting a full interval.
        uint32_t nowMs = millis();
        if (startupCheckinPending || nowMs - lastCheckinMs >= intervalMs) {
            bool startup = startupCheckinPending;
            startupCheckinPending = false;
            lastCheckinMs = nowMs;
            sendMessage(FAMILYTRACKER_MSG_CHECKIN, 0, true, false, 0);
            meshtastic_PositionLite p;
            bool hp = false, st = false;
            uint8_t am = 0;
            fillBestPosition(&p, &hp, &st, &am);
            if (startup)
                FT_DEBUG("checkin tx STARTUP gps=%s", hp ? (st ? "stale" : "fresh") : "none");
            else
                LOG_INFO("FamilyTracker: checkin tx periodic gps=%s", hp ? (st ? "stale" : "fresh") : "none");
        }
        // Child-side parent watchdog (mirror of the parent missed-check-in): no
        // parent heard for the same timeout -> local come-back alert. Requires
        // prior contact so a freshly booted child never false-alarms.
        if (parentHeardEver && !parentMissingAlerted &&
            (uint32_t)(millis() - lastParentHeardMs) > missedTimeoutSecs * 1000UL) {
            parentMissingAlerted = true;
            playMarioMelody(); // come-back semantics: "head home / find parents"
            FT_DEBUG("no parent contact for %u min - local come back", missedTimeoutSecs / 60);
            LOG_WARN("FamilyTracker: no parent contact for %u min - local come back", missedTimeoutSecs / 60);
            if (screen)
                screen->showSimpleBanner("No parent contact!\nHead home", 10000);
        }
        // Child tracker default view (BUG-010/ENH-010): nearest-parent banner.
        if (screen && nowMs - lastNearestParentMs >= 30000UL) {
            lastNearestParentMs = nowMs;
            updateNearestParentDisplay();
        }

        // ENH-008: persistent own-alarm indication. Deliberately LAST in the
        // child tick: the SOS LED loop must never preempt the periodic CHECKIN
        // again - starving it was BUG-001's false missed-check-in source.
        bool alarming = panicState == FamilyPanicState::PANIC_ACTIVE || lostModeActive;
#if defined(PIN_LED1)
        indicateSosLed(alarming); // self-releases when alarming goes false
#endif
        if (alarming) {
#if HAS_SCREEN
            if (screen && millis() - lastAlarmBannerMs >= 8000UL) {
                lastAlarmBannerMs = millis();
                screen->showSimpleBanner(lostModeActive ? "LOST MODE" : "PANIC ACTIVE", 8000);
            }
#endif
            return 100; // fast tick while indicating
        }
        return 5000;
    }

    if (!isParent())
        return 5000;

    // Parent startup presence (BUG-011/ENH-007): announce + position once so a
    // child rebooted into an empty mesh learns about us promptly.
    if (startupPresencePending) {
        startupPresencePending = false;
        sendMessage(FAMILYTRACKER_MSG_PARENT_PRESENCE, 0, true, false, 0);
        FT_DEBUG("parent presence tx (startup)");
    }

    // Periodic parent beacon: a GPS-less parent indoors otherwise transmits
    // NOTHING for hours, which would trip the child's "no parent contact"
    // watchdog. A tiny presence datagram every 5 min keeps children informed
    // (contact proof + nearest-parent refresh).
    if (millis() - lastPresenceMs >= FAMILYTRACKER_PARENT_PRESENCE_INTERVAL_MS) {
        lastPresenceMs = millis();
        sendMessage(FAMILYTRACKER_MSG_PARENT_PRESENCE, 0, true, false, 0);
    }

    // Base/relay nodes don't run the parent watchdog: a silent sentinel that
    // only alerts on explicit PANIC/LOST CHILD, and must not duplicate the
    // parents' MISSED CHECK-IN broadcasts. A CLIENT_BASE that MOVES (adult
    // picked it up) promotes itself to a full CLIENT parent first.
    if (isBase()) {
        updateBaseMobility();
        if (!mobilePromoted)
            return 5000;
        LOG_INFO("FamilyTracker: promoted base now running full parent duties");
    }

    uint32_t timeoutSecs = missedTimeoutSecs;
    uint8_t lowBatPct = lowBatteryPct;

    size_t count = nodeDB->getNumMeshNodes();
    for (size_t i = 0; i < count; i++) {
        meshtastic_NodeInfoLite *n = nodeDB->getMeshNodeByIndex(i);
        if (!n)
            continue;
        if (n->num == nodeDB->getNodeNum())
            continue; // skip self
        // Only watch child-role nodes
        if (n->role != meshtastic_Config_DeviceConfig_Role_TRACKER)
            continue;

        // Session-aware watchdog (BUG-001, ARCH §8): a child only becomes
        // monitored after it has checked in THIS session. Never fall back to
        // persisted nodedb lastHeard - a stale pre-reboot timestamp must not
        // produce a false "missed check-in".
        uint32_t age = msSinceChildSeen(n->num);
        if (age == UINT32_MAX)
            continue;
        bool missed = age > (uint32_t)timeoutSecs * 1000UL;

        bool wasMissed = std::find(alertedMissed.begin(), alertedMissed.end(), n->num) != alertedMissed.end();
        if (missed && !wasMissed) {
            playMissedCheckinTone(); // dedicated tone (BUG-002) - not the generic boop
            LOG_WARN("FamilyTracker: CHILD 0x%08x MISSED CHECK-IN (%u s ago)", n->num, age);
            // One-reporter policy: if another parent already broadcast this
            // child's missed check-in, stay out of the group chat - the local
            // tone is our alert. Otherwise we report AND tell the other parents
            // to suppress their own texts (MISSED_ALERT datagram).
            bool suppressed;
            {
                concurrency::LockGuard guard(&stateLock);
                suppressed = ((int32_t)(missedSuppressedUntilMs[n->num] - millis()) > 0);
            }
            const char *cn = (n->long_name[0]) ? n->long_name : n->short_name;
            if (!suppressed) {
                sendTextAlert("MISSED CHECK-IN: %s missed check-in (%u s ago) - no contact", cn ? cn : "Child", age / 1000);
                meshtastic_MeshPacket *p = allocDataPacket();
                if (p) {
                    uint8_t payload[16] = {0};
                    payload[0] = FAMILYTRACKER_PROTOCOL_VERSION;
                    payload[1] = FAMILYTRACKER_MSG_MISSED_ALERT;
                    payload[12] = n->num & 0xFF;
                    payload[13] = (n->num >> 8) & 0xFF;
                    payload[14] = (n->num >> 16) & 0xFF;
                    payload[15] = (n->num >> 24) & 0xFF;
                    p->decoded.payload.size = 16;
                    memcpy(p->decoded.payload.bytes, payload, 16);
                    service->sendToMesh(p, RX_SRC_LOCAL, true);
                }
                {
                    concurrency::LockGuard guard2(&stateLock);
                    missedSuppressedUntilMs[n->num] = millis() + FAMILYTRACKER_MISSED_SUPPRESS_MS;
                }
            }
            alertedMissed.push_back(n->num);
        } else if (!missed && wasMissed) {
            // Recovery (SPEC §21) - clears quickly after the startup sync check-in
            LOG_INFO("FamilyTracker: CHILD 0x%08x check-in resumed", n->num);
            const char *cn = (n->long_name[0]) ? n->long_name : n->short_name;
            sendTextAlert("CHECK-IN RESUMED: %s check-in resumed - contact restored", cn ? cn : "Child");
            alertedMissed.erase(std::remove(alertedMissed.begin(), alertedMissed.end(), n->num), alertedMissed.end());
            {
                concurrency::LockGuard guard2(&stateLock);
                missedSuppressedUntilMs.erase(n->num);
            }
        }

        // Low battery from stored telemetry (SPEC §23/§24)
        meshtastic_DeviceMetrics metrics;
        bool hasMetrics = nodeDB->copyNodeTelemetry(n->num, metrics);
        bool lowBat = hasMetrics && metrics.has_battery_level && metrics.battery_level < lowBatPct;
        if (lowBat) {
            bool wasBat = std::find(alertedBattery.begin(), alertedBattery.end(), n->num) != alertedBattery.end();
            if (!wasBat) {
                playLowBatteryTone(); // dedicated tone - not the generic beep
                LOG_WARN("FamilyTracker: CHILD 0x%08x LOW BATTERY %u%%", n->num, metrics.battery_level);
                const char *cn = (n->long_name[0]) ? n->long_name : n->short_name;
                sendTextAlert("LOW BATTERY: %s low battery %u%%", cn ? cn : "Child", metrics.battery_level);
                alertedBattery.push_back(n->num);
            }
        } else {
            alertedBattery.erase(std::remove(alertedBattery.begin(), alertedBattery.end(), n->num), alertedBattery.end());
        }
    }

    return 5000;
}

bool FamilyTrackerModule::getNearestParent(const char **name, float *distM, int *bearingDeg)
{
    const meshtastic_NodeInfoLite *self = nodeDB->getMeshNode(nodeDB->getNodeNum());
    if (!self || !nodeDB->hasValidPosition(self)) {
        *name = nullptr;
        *distM = 0;
        *bearingDeg = 0;
        return false;
    }

    float selfLat = localPosition.latitude_i * 1e-7f;
    float selfLon = localPosition.longitude_i * 1e-7f;
    float bestDistM = INFINITY;
    int bestBearing = -1;
    const char *bestName = nullptr;
    size_t count = nodeDB->getNumMeshNodes();
    for (size_t i = 0; i < count; i++) {
        const meshtastic_NodeInfoLite *n = nodeDB->getMeshNodeByIndex(i);
        if (!n || n->num == nodeDB->getNodeNum())
            continue;
        // Only parents (non-tracker roles) are navigation targets - EXCEPT
        // stationary base nodes (CLIENT_BASE etc.): a camp relay shouldn't pull
        // the kids' arrow away from the parent actually coming for them. Once
        // the base is picked up it re-roles to CLIENT and its NodeInfo updates
        // everyone's nodedb, so it appears here again automatically.
        if (n->role == meshtastic_Config_DeviceConfig_Role_TRACKER ||
            n->role == meshtastic_Config_DeviceConfig_Role_TAK_TRACKER)
            continue;
        if (IS_ONE_OF(n->role, meshtastic_Config_DeviceConfig_Role_CLIENT_BASE,
                      meshtastic_Config_DeviceConfig_Role_ROUTER, meshtastic_Config_DeviceConfig_Role_ROUTER_CLIENT))
            continue;
        if (!nodeDB->hasValidPosition(n))
            continue;
        meshtastic_PositionLite nPos;
        if (!nodeDB->copyNodePosition(n->num, nPos))
            continue;
        float d = GeoCoord::latLongToMeter(selfLat, selfLon, nPos.latitude_i * 1e-7f, nPos.longitude_i * 1e-7f);
        if (d < bestDistM) {
            bestDistM = d;
            bestBearing = (int)GeoCoord::bearing(selfLat, selfLon, nPos.latitude_i * 1e-7f, nPos.longitude_i * 1e-7f);
            if (bestBearing < 0)
                bestBearing += 360;
            bestName = (n->long_name[0]) ? n->long_name : n->short_name;
        }
    }
    if (!bestName) {
        *name = nullptr;
        *distM = 0;
        *bearingDeg = 0;
        return false;
    }
    *name = bestName;
    *distM = bestDistM;
    *bearingDeg = bestBearing;
    return true;
}

void FamilyTrackerModule::autoFavourite(NodeNum num)
{
    if (!num || num == nodeDB->getNodeNum() || nodeDB->isFavorite(num))
        return;
    concurrency::LockGuard guard(&favouriteLock);
    if (std::find(favouriteQueued.begin(), favouriteQueued.end(), num) == favouriteQueued.end()) {
        favouriteQueued.push_back(num);
        pendingFavourites.push_back(num);
    }
}

// Child tracker warm-up hook (BUG-010/ENH-010): the nearest-parent ARROW now
// lives in a dedicated "Parent" screen frame (Screen::setFrames) instead of a
// text banner over the Position screen. Kept for the PARENT_PRESENCE path to
// seed logging; the frame itself computes live via getNearestParent().
void FamilyTrackerModule::updateNearestParentDisplay()
{
#if HAS_SCREEN
    if (!screen)
        return;
    const char *name = nullptr;
    float distM = 0;
    int bearing = 0;
    if (getNearestParent(&name, &distM, &bearing)) {
        char distStr[24];
        if (distM < 1000.0f)
            snprintf(distStr, sizeof(distStr), "%.0f m", distM);
        else
            snprintf(distStr, sizeof(distStr), "%.1f km", distM / 1000.0f);
        LOG_INFO("FamilyTracker: nearest parent = %s %s %d deg", name, distStr, bearing);
    } else {
        LOG_INFO("FamilyTracker: nearest parent unknown (no GPS or no parent with position)");
    }
#endif
}
