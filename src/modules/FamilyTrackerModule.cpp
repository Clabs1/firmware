#include "FamilyTrackerModule.h"
#include "GPS.h"
#include "NodeDB.h"
#include "main.h" // screen
#include "gps/RTC.h"
#include "gps/GeoCoord.h"
#include "buzz/buzz.h"
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

void FamilyTrackerModule::renderPanicAlert(NodeNum from, uint32_t eventId,
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
        // Defer the text send: sending TEXT_MESSAGE_APP from inside the RX path
        // makes the packet loop back and re-enter the message store, whose flash
        // save nests spiLock and hangs the (T1) parent. Queue for runOnce().
        snprintf(pendingAlert, sizeof(pendingAlert), "%s pressed the panic button at %s - %s away, %d deg (%s)",
                 childName, timeStr, distStr, bearingDeg, ageStr);
        LOG_WARN("FamilyTracker: PANIC event=%u from %s (%s) - %.1f m away %d° (%s)", eventId, childName,
                 (uint16_t)from, distM, bearingDeg, ageStr);
    } else {
        snprintf(pendingAlert, sizeof(pendingAlert), "%s pressed the panic button at %s - %s", childName, timeStr, ageStr);
        LOG_WARN("FamilyTracker: PANIC event=%u from %s (%s) - %s", eventId, childName, (uint16_t)from, ageStr);
    }
    alertPending = true;
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
        playPanicCall(); // SENT tone - three short "call" notes (SPEC §33)

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

void FamilyTrackerModule::sendComeBack()
{
    // Parent -> all children: "come back now" (regroup). Broadcast so every child
    // hears the Mario tune + banner, not just one.
    sendMessageTo(FAMILYTRACKER_MSG_COME_BACK, 0, false, false, 0, NODENUM_BROADCAST);
    LOG_WARN("FamilyTracker: COME BACK broadcast by parent");
}

void FamilyTrackerModule::sendFound()
{
    // Parent concludes a lost/panic search: broadcast a stand-down so EVERY node
    // clears its state and plays the "level complete" success tone. The child that
    // was actually active announces "FOUND" once (see the CANCEL handler).
    sendMessageTo(FAMILYTRACKER_MSG_CANCEL, 0, false, false, 0, NODENUM_BROADCAST);
    LOG_WARN("FamilyTracker: FOUND stand-down broadcast by parent");
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

// Extract the child name after a command, trimming surrounding whitespace and
// trailing punctuation ("lost Alice!" -> "Alice").
static void extractName(const char *src, char *out, size_t outLen)
{
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
        sendComeBack();
        return true;
    }
    if (matchCommand(text, "found")) {
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
            sendTextAlert("No child named \"%s\" found", name);
            return true;
        }
        pickLostChild(); // bare "lost": pick from the checked-in children
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

void FamilyTrackerModule::sendLostChild(NodeNum target)
{
    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p)
        return;

    uint8_t payload[12] = {0};
    payload[0] = FAMILYTRACKER_PROTOCOL_VERSION;
    payload[1] = FAMILYTRACKER_MSG_LOST_CHILD;
    payload[8] = target & 0xFF;
    payload[9] = (target >> 8) & 0xFF;
    payload[10] = (target >> 16) & 0xFF;
    payload[11] = (target >> 24) & 0xFF;
    p->to = NODENUM_BROADCAST;
    p->decoded.payload.size = sizeof(payload);
    memcpy(p->decoded.payload.bytes, payload, sizeof(payload));
    service->sendToMesh(p, RX_SRC_LOCAL, true);

    // Regroup: everyone comes back when a child is reported lost.
    sendComeBack();

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

    char name[32];
    const char *src = nullptr;

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
        sendLostChild(target);
        return ProcessMessage::STOP;
    }
    sendTextAlert("No child named \"%s\" found", src);
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
        sendTextAlert("No children have checked in yet");
        return;
    }

    screen->showNodePicker("Select lost child", 60000, [](uint32_t num) {
        if (familyTrackerModule && num)
            familyTrackerModule->sendLostChild(num);
    });
    // set after showNodePicker (which resets the filter to "all nodes")
    graphics::NotificationRenderer::setNodePickerFilter(&lostChildList);
#endif
}

ProcessMessage FamilyTrackerModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    // Human-readable family commands ("Child 1 is lost") arrive as text.
    if (mp.decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP)
        return handleTextCommand(mp);

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
                // Queue the whole alert (banner + buzzer + ACK + text) for runOnce:
                // doing it here inside the radio RX handler contends with the
                // radio/flash SPI lock on the nRF52 and intermittently hangs.
                pendingPanic.active = true;
                pendingPanic.from = mp.from;
                pendingPanic.eventId = eventId;
                pendingPanic.pos = pos;
                pendingPanic.stale = (flags & FAMILYTRACKER_FLAG_POS_STALE) != 0;
                pendingPanic.ageMin = ageMin;
                pendingPanic.hasPos = (flags & FAMILYTRACKER_FLAG_HAS_POS) != 0;
            }
        }
        break;

    case FAMILYTRACKER_MSG_PANIC_ACK:
        // Our panic was acknowledged: ACKED tone (SPEC §34)
        // Multi-Child (§18A): ACKs are now targeted at the specific child (mp.to
        // == us), so eventId alone can no longer collide with another child's
        // locally-numbered event. Accept targeted ACKs, and broadcast ACKs only
        // when they match our own outstanding panic (backwards compat).
        if (isChild() && eventId == lastPanicEventId && (mp.to == nodeDB->getNodeNum() || isBroadcast(mp.to))) {
            playPanicResponse(); // rising "response" completes the call/response (SPEC §34)
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

    case FAMILYTRACKER_MSG_COME_BACK:
        // Parent -> child: "come back now". Distinct tone + on-screen message.
        if (isChild() && (mp.to == nodeDB->getNodeNum() || isBroadcast(mp.to))) {
            playMarioMelody(); // distinct "come back" tune (SPEC §34A)
            if (screen)
                screen->showSimpleBanner("Come back now!", 8000);
            LOG_INFO("FamilyTracker: COME BACK from 0x%08x", mp.from);
        }
        break;

    case FAMILYTRACKER_MSG_LOST_CHILD: {
        // Parent -> group: "child X is lost". Target node rides in payload
        // bytes 8-11 (uint32 LE). The target child enters lost mode (fast
        // check-in) and announces itself; parents raise a local alert.
        uint32_t target = 0;
        if (mp.decoded.payload.size >= 12) {
            const uint8_t *b = mp.decoded.payload.bytes;
            target = (uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16) | ((uint32_t)b[11] << 24);
        }
        if (isChild() && target == nodeDB->getNodeNum()) {
            lostModeActive = true; // persists until a parent sends FOUND
            playSosTone(); // SOS in morse - "you are lost"
            if (screen)
                screen->showSimpleBanner("Lost child - stay where you are", 15000);
            char lost[64];
            snprintf(lost, sizeof(lost), "%s is lost - please help find me", owner.short_name);
            sendTextAlert("%s", lost);
            LOG_WARN("FamilyTracker: LOST CHILD (me) - entering lost mode");
        } else if (isParent()) {
            const meshtastic_NodeInfoLite *c = nodeDB->getMeshNode(target);
            const char *cn = "Child";
            if (c && c->long_name[0])
                cn = c->long_name;
            else if (c && c->short_name[0])
                cn = c->short_name;
            playLostAlert(); // distinct from the child's SOS
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
            // Only the node that actually had an active alert announces "found"
            // (so a broadcast stand-down doesn't make every node claim it was
            // found). Everyone else just stands down with the success tone.
            bool wasActive = findSoundUntilMs != 0 || lostModeActive || (isChild() && lastPanicEventId != 0);
            findSoundUntilMs = 0;
            lostModeActive = false;
            lastPanicEventId = 0; // stand down any outstanding panic
            playFoundMelody();    // "level complete" success tone
            if (screen)
                screen->showSimpleBanner("Found - standing down", 8000);
            if (wasActive)
                sendTextAlert("FOUND: %s found by %s - standing down", owner.short_name, pn);
            LOG_WARN("FamilyTracker: FOUND by %s (0x%08x)", pn, mp.from);
        }
        break;

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
            // A button press while the find sound is active cancels it (and does
            // NOT raise a panic) - so a child/parent can silence "find me" once
            // found without accidentally triggering a panic.
            if (findSoundUntilMs) {
                findSoundUntilMs = 0;
                LOG_INFO("FamilyTracker: FIND SOUND cancelled by button");
                break;
            }
            if (isChild()) {
                uint32_t eventId = nextEventId++;
                lastPanicEventId = eventId; // so the returning ACK matches (SPEC §34)
                sendMessage(FAMILYTRACKER_MSG_PANIC, eventId, true, false, 0);
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
    // Act on a queued panic OUTSIDE the radio RX path: banner + buzzer + ACK.
    // renderPanicAlert() in turn queues the human-readable text (alertPending),
    // flushed on the next tick. None of these side-effects run inside
    // handleReceived(), which is what hangs the T1 parent.
    if (pendingPanic.active) {
        pendingPanic.active = false;
        renderPanicAlert(pendingPanic.from, pendingPanic.eventId, pendingPanic.pos, pendingPanic.stale,
                         pendingPanic.ageMin, pendingPanic.hasPos);
        playPanicAlert(); // alert (distinct from the child's call)
        sendAck(pendingPanic.eventId, pendingPanic.from); // SPEC §34 — targeted ACK (multi-Child §18A)
        return 200; // fast tick so the queued text flushes promptly
    }

    // Flush a deferred panic alert (queued by renderPanicAlert to avoid a
    // reentrant message-store write that hangs the T1 parent).
    if (alertPending) {
        alertPending = false;
        sendTextAlert("%s", pendingAlert);
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
        static uint32_t lastCheckinMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastCheckinMs >= intervalMs) {
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
