#include "FamilyTrackerModule.h"
#include "GPS.h"
#include "NodeDB.h"
#include "main.h" // screen
#include "gps/RTC.h"
#include "gps/GeoCoord.h"
#include "buzz/buzz.h"
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

void FamilyTrackerModule::setup()
{
    LOG_WARN("FamilyTrackerModule: armed (role=%s)", isChild() ? "child" : "parent");
}

void FamilyTrackerModule::buzzerBeep(bool ack)
{
    if (ack) {
        playBoop();
    } else {
        playBeep();
    }
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
    uint32_t nowSecs = getValidTime(RTCQuality::RTCQualityDevice, true);
    if (fixTime > 0 && nowSecs > fixTime) {
        uint32_t ageSecs = nowSecs - fixTime;
        if (ageSecs > FAMILYTRACKER_POSITION_FRESH_SECS) {
            *stale = true;
            *ageMin = (uint8_t)(ageSecs / 60);
        }
    }
    *hasPos = true;
}

void FamilyTrackerModule::renderPanicAlert(const meshtastic_MeshPacket &mp, uint32_t eventId,
                                           const meshtastic_PositionLite &pos, bool stale, uint8_t ageMin, bool hasPos)
{
    // Child identity (SPEC §18A multi-child): name from the child's nodedb entry.
    const char *childName = nullptr;
    const meshtastic_NodeInfoLite *child = nodeDB->getMeshNode(mp.from);
    if (child && child->long_name[0]) {
        childName = child->long_name;
    } else if (child && child->short_name[0]) {
        childName = child->short_name;
    }
    char nameBuf[16];
    if (!childName) {
        snprintf(nameBuf, sizeof(nameBuf), "0x%04x", (uint16_t)mp.from);
        childName = nameBuf;
    }

    // Panic time: our device RTC at receipt (hh:mm).
    char timeStr[8] = "--:--";
    uint32_t nowSecs = getValidTime(RTCQuality::RTCQualityDevice, true); // already local (TZ applied)
    if (nowSecs) {
        uint8_t hh = (nowSecs / 3600) % 24;
        uint8_t mm = (nowSecs / 60) % 60;
        snprintf(timeStr, sizeof(timeStr), "%02u:%02u", hh, mm);
    }

    // Distance + bearing from the parent's own position (SPEC §35 geo distance,
    // never RSSI/hops) to the child's reported position.
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
        sendTextAlert("%s pressed the panic button at %s - %s away, %d deg (%s)", childName, timeStr, distStr,
                      bearingDeg, ageStr);
        LOG_WARN("FamilyTracker: PANIC event=%u from %s (%s) - %.1f m away %d° (%s)", eventId, childName,
                 (uint16_t)mp.from, distM, bearingDeg, ageStr);
    } else {
        sendTextAlert("%s pressed the panic button at %s - %s", childName, timeStr, ageStr);
        LOG_WARN("FamilyTracker: PANIC event=%u from %s (%s) - %s", eventId, childName, (uint16_t)mp.from, ageStr);
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

    uint8_t payload[21] = {0};
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
    size_t len = 8;
    if (hasPos) {
        payload[8] = pos.latitude_i & 0xFF;
        payload[9] = (pos.latitude_i >> 8) & 0xFF;
        payload[10] = (pos.latitude_i >> 16) & 0xFF;
        payload[11] = (pos.latitude_i >> 24) & 0xFF;
        payload[12] = pos.longitude_i & 0xFF;
        payload[13] = (pos.longitude_i >> 8) & 0xFF;
        payload[14] = (pos.longitude_i >> 16) & 0xFF;
        payload[15] = (pos.longitude_i >> 24) & 0xFF;
        payload[16] = pos.altitude & 0xFF;
        payload[17] = (pos.altitude >> 8) & 0xFF;
        len = 18;
    }

    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p)
        return;
    p->to = to;
    p->decoded.payload.size = len;
    memcpy(p->decoded.payload.bytes, payload, len);

    service->sendToMesh(p, RX_SRC_LOCAL, true);

    if (msgType == FAMILYTRACKER_MSG_PANIC)
        buzzerBeep(false); // SENT tone (SPEC §33)

    LOG_INFO("FamilyTracker: TX msg=%u event=%u to=%04x%s%s", msgType, eventId, (uint16_t)to, hasPos ? " pos" : "",
             stale ? " (stale)" : "");
}

void FamilyTrackerModule::sendAck(uint32_t eventId, NodeNum to)
{
    // Targeted ACK to the child that raised the panic (multi-Child §18A):
    // prevents a broadcast ACK from being matched by a different child that
    // happened to use the same eventId (each child numbers events locally).
    sendMessageTo(FAMILYTRACKER_MSG_PANIC_ACK, eventId, false, false, 0, to);
}

void FamilyTrackerModule::sendOnWay(uint32_t eventId, uint8_t presetIndex, NodeNum to)
{
    if (presetIndex >= FAMILYTRACKER_ON_WAY_COUNT)
        presetIndex = 0;
    const char *msg = familyTrackerOnWayMessages[presetIndex];

    // 1) Human-readable Family Channel text (SPEC §34A) so the whole family
    //    (and any parent console/app) sees who responded and what they said.
    char buf[128];
    snprintf(buf, sizeof(buf), "%s: %s", owner.short_name, msg);
    sendTextAlert("%s", buf);

    // 2) PARENT_ON_WAY datagram, targeted to the child: distinct tone + on-screen
    //    message at the child. presetIndex rides in the ageMin byte.
    sendMessageTo(FAMILYTRACKER_MSG_PARENT_ON_WAY, eventId, false, false, presetIndex, to);
    LOG_INFO("FamilyTracker: PARENT_ON_WAY event=%u to=%04x preset=%u '%s'", eventId, (uint16_t)to, presetIndex, msg);
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

bool FamilyTrackerModule::isValidMessage(const meshtastic_MeshPacket &mp, uint8_t *msgType, uint32_t *eventId,
                                         uint8_t *flags, uint8_t *ageMin, meshtastic_PositionLite *pos)
{
    const uint8_t *b = mp.decoded.payload.bytes;
    if (mp.decoded.payload.size < 8)
        return false;
    if (b[0] != FAMILYTRACKER_PROTOCOL_VERSION)
        return false;

    *msgType = b[1];
    *eventId = (uint32_t)b[2] | ((uint32_t)b[3] << 8) | ((uint32_t)b[4] << 16) | ((uint32_t)b[5] << 24);
    *flags = b[6];
    *ageMin = b[7];
    memset(pos, 0, sizeof(*pos));

    if ((*flags & FAMILYTRACKER_FLAG_HAS_POS) && mp.decoded.payload.size >= 18) {
        pos->latitude_i = (int32_t)((uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16) | ((uint32_t)b[11] << 24));
        pos->longitude_i = (int32_t)((uint32_t)b[12] | ((uint32_t)b[13] << 8) | ((uint32_t)b[14] << 16) |
                                     ((uint32_t)b[15] << 24));
        pos->altitude = (int16_t)((uint16_t)b[16] | ((uint16_t)b[17] << 8));
    }
    return true;
}

ProcessMessage FamilyTrackerModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    uint8_t msgType, flags, ageMin;
    uint32_t eventId;
    meshtastic_PositionLite pos;
    if (!isValidMessage(mp, &msgType, &eventId, &flags, &ageMin, &pos))
        return ProcessMessage::CONTINUE;

    switch (msgType) {
    case FAMILYTRACKER_MSG_PANIC:
        // A child panicked. Parents alert + ACK; children ignore (only one parent needed).
        lastPanicEventId = eventId;
        if (isParent()) {
            markChildSeen(mp.from, millis()); // panic is also proof-of-life (SPEC §18)
            bool alreadyAlerted =
                std::find(alertedPanic.begin(), alertedPanic.end(), mp.from) != alertedPanic.end();
            if (!alreadyAlerted) {
                alertedPanic.push_back(mp.from);
                renderPanicAlert(mp, eventId, pos, (flags & FAMILYTRACKER_FLAG_POS_STALE) != 0, ageMin,
                                 (flags & FAMILYTRACKER_FLAG_HAS_POS) != 0);
            }
            buzzerBeep(true); // alert
            sendAck(eventId, mp.from); // SPEC §34 — targeted to the panicking child (multi-Child §18A)
        }
        break;

    case FAMILYTRACKER_MSG_PANIC_ACK:
        // Our panic was acknowledged: ACKED tone (SPEC §34)
        // Multi-Child (§18A): ACKs are now targeted at the specific child (mp.to
        // == us), so eventId alone can no longer collide with another child's
        // locally-numbered event. Accept targeted ACKs, and broadcast ACKs only
        // when they match our own outstanding panic (backwards compat).
        if (isChild() && eventId == lastPanicEventId && (mp.to == nodeDB->getNodeNum() || isBroadcast(mp.to))) {
            buzzerBeep(true);
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
            uint32_t pe = nextEventId++;
            lastPanicEventId = pe; // so the returning ACK matches (SPEC §34)
            sendMessage(FAMILYTRACKER_MSG_PANIC, pe, true, false, 0);
            char timeStr[8] = "--:--";
            uint32_t nowSecs = getValidTime(RTCQuality::RTCQualityDevice, true); // already local (TZ applied)
            if (nowSecs) {
                snprintf(timeStr, sizeof(timeStr), "%02u:%02u", (nowSecs / 3600) % 24, (nowSecs / 60) % 60);
            }
            sendTextAlert("PANIC sent by %s at %s - event %u", owner.short_name, timeStr, pe);
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
        LOG_INFO("FamilyTracker: CHECKIN from 0x%08x%s%s", mp.from,
                 (flags & FAMILYTRACKER_FLAG_HAS_POS) ? " pos" : " (no pos)",
                 (flags & FAMILYTRACKER_FLAG_POS_STALE) ? " (stale)" : "");
        break;

#ifdef FAMILY_TEST_HOOKS
    case FAMILYTRACKER_MSG_CONFIG: {
        // Remote test-config: set the parent watchdog timeout + low-battery
        // threshold so §18/§20/§23 can be exercised without a 10-min wait.
        // payload byte 7 = timeout in seconds (0 = leave unchanged),
        // byte 8   = low-battery percent (0 = leave unchanged).
        if (isParent() && mp.decoded.payload.size >= 9) {
            uint8_t t = mp.decoded.payload.bytes[7];
            uint8_t b = mp.decoded.payload.bytes[8];
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

    default:
        return ProcessMessage::CONTINUE;
    }
    return ProcessMessage::STOP;
}

int FamilyTrackerModule::handleInputEvent(const InputEvent *event)
{
    if (event) {
        switch (event->inputEvent) {
        case INPUT_BROKER_USER_PRESS:
        case INPUT_BROKER_SEND_PING:
        case INPUT_BROKER_ALT_PRESS:
            if (isChild()) {
                uint32_t eventId = nextEventId++;
                lastPanicEventId = eventId; // so the returning ACK matches (SPEC §34)
                sendMessage(FAMILYTRACKER_MSG_PANIC, eventId, true, false, 0);
                char timeStr[8] = "--:--";
                uint32_t nowSecs = getValidTime(RTCQuality::RTCQualityDevice, true); // already local (TZ applied)
                if (nowSecs) {
                    snprintf(timeStr, sizeof(timeStr), "%02u:%02u", (nowSecs / 3600) % 24, (nowSecs / 60) % 60);
                }
                sendTextAlert("PANIC sent by %s at %s - event %u", owner.short_name, timeStr, eventId);
            }
            break;
        default:
            break;
        }
    }
    return 0;
}

// Parent watchdog: missed check-in + low battery (SPEC §18-§23).
// Child: periodic CHECKIN heartbeat independent of GPS (SPEC §13/§14/§18).
int32_t FamilyTrackerModule::runOnce()
{
    if (isChild()) {
        // Periodic check-in so the parent can monitor us even without a GPS fix.
        // Use millis() (monotonic) not getValidTime() so it works without an RTC.
        static uint32_t lastCheckinMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastCheckinMs >= (uint32_t)FAMILYTRACKER_CHECKIN_INTERVAL_SECS * 1000UL) {
            lastCheckinMs = nowMs;
            sendMessage(FAMILYTRACKER_MSG_CHECKIN, 0, true, false, 0);
        }
        return 5000;
    }

    if (!isParent())
        return 5000;

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

        uint32_t age = msSinceChildSeen(n->num);
        if (age == UINT32_MAX) {
            // Never seen via our protocol; fall back to nodedb lastHeard so a
            // stock-broadcast child (position/nodeinfo only) is still monitored.
            age = sinceLastSeen(n);
        }
        bool missed = (age != UINT32_MAX) && (age > (uint32_t)timeoutSecs * 1000UL);

        bool wasMissed = std::find(alertedMissed.begin(), alertedMissed.end(), n->num) != alertedMissed.end();
        if (missed && !wasMissed) {
            buzzerBeep(true);
            LOG_WARN("FamilyTracker: CHILD 0x%08x MISSED CHECK-IN (%u s ago)", n->num, age);
            const char *cn = (n->long_name[0]) ? n->long_name : n->short_name;
            sendTextAlert("MISSED CHECK-IN: %s missed check-in (%u s ago) - no contact", cn ? cn : "Child", age / 1000);
            alertedMissed.push_back(n->num);
        } else if (!missed && wasMissed) {
            // Recovery (SPEC §21)
            LOG_INFO("FamilyTracker: CHILD 0x%08x check-in resumed", n->num);
            const char *cn = (n->long_name[0]) ? n->long_name : n->short_name;
            sendTextAlert("CHECK-IN RESUMED: %s check-in resumed - contact restored", cn ? cn : "Child");
            alertedMissed.erase(std::remove(alertedMissed.begin(), alertedMissed.end(), n->num), alertedMissed.end());
        }

        // Low battery from stored telemetry (SPEC §23/§24)
        meshtastic_DeviceMetrics metrics;
        bool hasMetrics = nodeDB->copyNodeTelemetry(n->num, metrics);
        bool lowBat = hasMetrics && metrics.has_battery_level && metrics.battery_level < lowBatPct;
        if (lowBat) {
            bool wasBat = std::find(alertedBattery.begin(), alertedBattery.end(), n->num) != alertedBattery.end();
            if (!wasBat) {
                buzzerBeep(false);
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
