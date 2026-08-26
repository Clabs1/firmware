#pragma once

#include "MeshModule.h"
#include "MeshService.h"
#include "SinglePortModule.h"
#include "configuration.h"
#include "concurrency/OSThread.h"
#include "concurrency/Lock.h"
#include "concurrency/LockGuard.h"
#include "input/InputBroker.h"
#include <map>
#include <set>
#include <vector>

/**
 * Family Tracker module - child safety panic + locate + parent watchdog.
 *
 * One firmware, dual role (SPEC §8/§31): behaviour is driven by
 * config.device.role at runtime.
 *
 * CHILD (TRACKER role):
 *   - Button press  -> broadcast PANIC (always, regardless of GPS; carries best
 *                      position: live, or stale <=15 min flagged, or none)
 *   - LOCATE_REQ rx -> locate tone + immediate LOCATE_RESP (latest position),
 *                      then trigger a fresh GPS fix and send updated LOCATE_RESP
 *   - PANIC_ACK rx  -> ACKED tone
 *
 * PARENT (CLIENT / ROUTER_CLIENT role):
 *   - PANIC rx      -> alert + broadcast PANIC_ACK
 *   - missed-check-in watchdog: per-child, alert if sinceLastSeen > timeout
 *   - low-battery alert from child telemetry
 *
 * Payload v0.3 (max 22 bytes):
 *   byte 0      : protocol version (0x03 = v0.3, stays < v1)
 *   byte 1      : msg type
 *   bytes 2-5   : event id (uint32 LE)
 *   byte 6      : flags (bit0 HAS_POS, bit1 POS_STALE)
 *   byte 7      : pos_age_minutes
 *   bytes 8-11  : event timestamp (uint32 LE, UTC epoch secs) - child-authoritative
 *   bytes 12-15 : latitude_i (int32 LE)   [if HAS_POS]
 *   bytes 16-19 : longitude_i (int32 LE)  [if HAS_POS]
 *   bytes 20-21 : altitude (int16 LE)     [if HAS_POS]
 */
#define FAMILYTRACKER_PROTOCOL_VERSION 0x03

#define FAMILYTRACKER_MSG_PANIC       0x01
#define FAMILYTRACKER_MSG_PANIC_ACK   0x02
#define FAMILYTRACKER_MSG_LOCATE_REQ  0x03
#define FAMILYTRACKER_MSG_LOCATE_RESP 0x04
#define FAMILYTRACKER_MSG_CHECKIN     0x05
#define FAMILYTRACKER_MSG_PANIC_TRIGGER 0x06
#define FAMILYTRACKER_MSG_CONFIG      0x07  // set watchdog timeout / low-battery % (remote test config)
#define FAMILYTRACKER_MSG_PARENT_ON_WAY  0x08  // parent -> child: "read + on the way" (preselected msg)
#define FAMILYTRACKER_MSG_ON_WAY_TRIGGER 0x09  // remote trigger for hands-off testing (like PANIC_TRIGGER)
#define FAMILYTRACKER_MSG_COME_BACK     0x0A  // parent -> child: "come back now" (tone + banner)
#define FAMILYTRACKER_MSG_LOST_CHILD    0x0B  // parent -> group: "child X is lost" (target node in payload)
#define FAMILYTRACKER_MSG_FIND_SOUND    0x0C  // any -> node: loud repeating find tone
#define FAMILYTRACKER_MSG_CANCEL        0x0D  // any -> node: cancel active find-sound / lost-mode / panic
#define FAMILYTRACKER_MSG_PARENT_PRESENCE 0x0E // parent -> group: startup presence + position (BUG-011)
#define FAMILYTRACKER_MSG_MISSED_ALERT  0x0F // parent -> group: "I reported child X missed" (spam suppression;
                                             // target child num in payload bytes 12-15, LOST_CHILD layout)

// Preselected "on the way" response messages (SPEC §34A). The parent picks one
// and it is sent to the child as a human-readable Family Channel text plus a
// PARENT_ON_WAY datagram that triggers a distinct tone + on-screen message at
// the child.
#define FAMILYTRACKER_ON_WAY_COUNT 4
#define FAMILYTRACKER_ON_WAY_MSG_MAX 32

#define FAMILYTRACKER_FLAG_HAS_POS   0x01
#define FAMILYTRACKER_FLAG_POS_STALE 0x02

#define FAMILYTRACKER_POSITION_FRESH_SECS (15 * 60)

// Parent watchdog defaults (SPEC §18/§19/§22)
#define FAMILYTRACKER_DEFAULT_MISSED_TIMEOUT_SECS (10 * 60)
#define FAMILYTRACKER_DEFAULT_LOW_BATTERY_PCT    20

// Parent-triggered flows (parent -> child/group)
#define FAMILYTRACKER_LOST_CHECKIN_SECS     10         // fast check-in cadence while lost
#define FAMILYTRACKER_FIND_SOUND_SECS       150        // find tone auto-stops after 2m30s
#define FAMILYTRACKER_FIND_BEEP_INTERVAL_MS 2000       // re-beep cadence for the find sound

// Child periodic check-in cadence (SPEC §13/§14/§18) - independent of GPS so the
// parent can always distinguish "idle" from "missing".
#define FAMILYTRACKER_CHECKIN_INTERVAL_SECS 120

// Parent presence beacon cadence: proof-of-contact for the child watchdog even
// when parents have no GPS fix (and would otherwise stay radio-silent indoors).
#define FAMILYTRACKER_PARENT_PRESENCE_INTERVAL_MS (5 * 60 * 1000UL)

// After another parent reports a child missed, suppress our own group report for
// this long (one reporter policy; the tone still plays locally).
#define FAMILYTRACKER_MISSED_SUPPRESS_MS (10 * 60 * 1000UL)

// Base/relay mobility threshold: displacement from the boot anchor that counts
// as "an adult picked the camp/car node up and is joining the search".
#define FAMILYTRACKER_BASE_MOVE_METRES 150.0f

// Panic state machine (ARCH §2). Minimum time between panic triggers on the
// child (retrigger permitted after this cooldown, fresh event ID) and the
// dedup window for the SAME (from, eventId) on a parent (mesh retransmissions
// must not re-alert).
#define FAMILYTRACKER_PANIC_COOLDOWN_MS 20000
#define FAMILYTRACKER_PANIC_DEDUP_MS    60000

enum class FamilyPanicState : uint8_t { NORMAL, PANIC_ACTIVE, PANIC_CLEARED };

class FamilyTrackerModule : public SinglePortModule, public concurrency::OSThread
{
  public:
    // __attribute__((used)) prevents whole-image LTO (nrf52_lto.py) from treating the
    // constructor call in setupModules() as dead code, since the result is discarded.
    __attribute__((used)) FamilyTrackerModule();

    bool wantPacket(const meshtastic_MeshPacket *p) override
    {
        return p->decoded.portnum == meshtastic_PortNum_PRIVATE_APP ||
               p->decoded.portnum == meshtastic_PortNum_TEXT_MESSAGE_APP;
    }

    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

    void setup() override;

    int handleInputEvent(const InputEvent *event);

    // Role helpers. Public so InputBroker can route the screen-wake press to a
    // child (BUG-014) and null the screenless triple-press GPS toggle (ENH-003).
    bool isChild() const;
    bool isParent() const;
    // Base/relay node (ROUTER / ROUTER_CLIENT): stationary mesh extender left in
    // a central high spot. Acts as a family parent but keeps its buzzer silent
    // except for PANIC / LOST CHILD, so a nearby adult notices an emergency and
    // can grab the node to join the search.
    bool isBase() const;

    // Screen-equipped child: a press with the display asleep only wakes the
    // screen (shows the arrow-to-parent frame); panic needs a further press once
    // the screen is on (revert of BUG-014/ENH-011 wake-press panic).
    // Screenless builds have no wake state, so the first press still panics.
    bool consumesWakeupPress() const
    {
#if HAS_SCREEN
        return false;
#else
        return isChild();
#endif
    }

    // Handle a family command sent as a canned/free text (e.g. "come back",
    // "found", "lost <name>"). Returns true if the text matched a command and triggered an action.
    bool handleFamilyCommand(const char *text);

    // Parent declares a child lost: broadcasts LOST_CHILD (target in payload) and
    // also broadcasts COME_BACK so the whole group regroups. SPEC §31/§35.
    void sendLostChild(NodeNum target);

    // Nearest-parent navigation data for the child's "Parent" screen frame
    // (BUG-010/ENH): returns true and fills name/distance (m)/bearing (deg) when
    // a non-tracker node with a valid position is known. Live-computed so the
    // frame always shows current data.
    bool getNearestParent(const char **name, float *distM, int *bearingDeg);

    int32_t runOnce() override; // OSThread: parent watchdog tick

    CallbackObserver<FamilyTrackerModule, const InputEvent *> inputObserver =
        CallbackObserver<FamilyTrackerModule, const InputEvent *>(this, &FamilyTrackerModule::handleInputEvent);

  private:
    void sendMessage(uint8_t msgType, uint32_t eventId, bool hasPos, bool stale, uint8_t ageMin);
    void sendMessageTo(uint8_t msgType, uint32_t eventId, bool hasPos, bool stale, uint8_t ageMin, NodeNum to);
    void sendTextAlert(const char *format, ...);
    // sendAck: acknowledge the panic to the specific child that raised it
    void sendAck(uint32_t eventId, NodeNum to);

    void buzzerBeep(bool ack);
    bool isValidMessage(const meshtastic_MeshPacket &mp, uint8_t *msgType, uint32_t *eventId, uint32_t *eventTs,
                        uint8_t *flags, uint8_t *ageMin, meshtastic_PositionLite *pos);
    void fillBestPosition(meshtastic_PositionLite *pos, bool *hasPos, bool *stale, uint8_t *ageMin);
    void triggerFreshFix();

    // Child panic send: enforces the cooldown, assigns a fresh event ID, moves
    // to PANIC_ACTIVE, and on the first trigger also broadcasts RETURN_TO_PARENT
    // (COME_BACK) so sibling children regroup (BUG-008/009/012/013, ARCH §2/§3).
    void sendPanic();

    // Human-readable panic rendering (SPEC §34 "family channel human-readable")
    // returns "<name> pressed the panic button at hh:mm - <dist> <dir> (<age>)"
    void renderPanicAlert(NodeNum from, uint32_t eventId, uint32_t eventTs, const meshtastic_PositionLite &pos,
                          bool stale, uint8_t ageMin, bool hasPos);

    // Send the preselected "on the way" response to a specific child: a Family
    // Channel text (human-readable) + PARENT_ON_WAY datagram (distinct tone +
    // on-screen message at the child). SPEC §34A. announce=false re-fires
    // another parent's response silently.
    void sendOnWay(uint32_t eventId, uint8_t presetIndex, NodeNum to, bool announce = true);

    // Deliver a text message to our own phone (self-addressed, never leaves the
    // node over LoRa) so screenless parents see an error in their chat view.
    void notifySelfText(const char *fmt, ...);

    // Parent -> all children: "come back now" (regroup) broadcast, announced in chat.
    void sendComeBack();

    // Parent -> one child: "come back now" targeted at a single tracker.
    void sendComeBackTo(NodeNum target);

    // Child-originated silent regroup (ENH-007): datagram only, no chat text.
    void sendComeBackRegroup();

    // Shared core: COME_BACK datagram to `to`, optionally announcing in chat.
    void sendComeBackMsg(NodeNum to, bool announce);

    // Parent concludes a lost/panic search: stand-down broadcast + success tone.
    void sendFound();

    // Parent -> ONE child: stand-down targeted at a single tracker ("found bob"
    // clears only Bob). Returns true if the target resolved and was cleared.
    bool sendFoundTo(NodeNum target);

    // Names of children THIS parent declared lost and that are still out
    // (comma-joined into out). Used for "STILL MISSING" follow-ups.
    size_t stillMissingNames(char *out, size_t outLen);

    // Look up a tracker-role child by long_name or short_name (case-insensitive).
    // Returns NODENUM_BROADCAST (0) if none matched.
    NodeNum findChildByName(const char *name);

    // Look up ANY family node by name (children, parents, base) - used by the
    // find-sound command so a dropped base/car unit can chirp too.
    NodeNum findAnyNodeByName(const char *name);

    // Trigger the loud repeating find tone on a specific node ("dropped in the
    // grass"). Any button press on that node cancels it early. announce=false
    // when re-firing another parent's command (no duplicate chat text).
    void sendFindSound(NodeNum to, bool announce = true);

    // Children THIS parent declared lost via targeted "lost <name>".
    std::set<NodeNum> lostDeclaredByUs;

    // Handle a human-readable family command received as TEXT_MESSAGE_APP
    // (e.g. "Child 1 is lost"). Returns STOP if consumed, else CONTINUE.
    ProcessMessage handleTextCommand(const meshtastic_MeshPacket &mp);

    // Local-only feedback when a command names a child we can't resolve (typo):
    // error tone + banner on THIS node, never broadcast to the channel. The
    // sending parent always resolves their own text via the local-broadcast
    // loopback, so the typo-ing node is the one that beeps.
    void unknownChildError(const char *name);

    // Open a tracker-only node picker so the parent can select which child is lost.
    void pickLostChild();

    // Parent watchdog state
    std::vector<NodeNum> alertedMissed;
    std::vector<NodeNum> alertedBattery;
    // Panic dedup by (from, eventId) with a time window (BUG-003/004/008): a NEW
    // event ID always alerts again (retrigger), the SAME event ID within the
    // dedup window (mesh retransmission) is ignored. Never a permanent per-node
    // latch, so a child can panic repeatedly in one session.
    std::map<NodeNum, uint32_t> lastPanicEventIdByNode;
    std::map<NodeNum, uint32_t> lastPanicAlertMsByNode;

    // Direct per-child last-seen (ms, monotonic millis()). NOT nodedb lastHeard:
    // our CHECKIN/PANIC are PRIVATE_APP packets that the router consumes (STOP)
    // and never updates nodedb lastHeard with, so a nodedb-based watchdog would
    // false-alert even though the child is CHECKINing every 2 min. Updated on any
    // child-originated FamilyTracker message (CHECKIN/PANIC/LOCATE_RESP/ON_WAY rx).
    std::map<NodeNum, uint32_t> childLastSeenMs;
    // childLastSeenMs is written by markChildSeen() on the Router thread
    // (handleReceived) and read by msSinceChildSeen() on this module's own
    // OSThread (runOnce watchdog). Without a lock the concurrent find/erase
    // corrupts the map and hangs the parent. Mutable so the const reader can
    // still take it.
    mutable concurrency::Lock childSeenLock;
    void markChildSeen(NodeNum from, uint32_t nowMs)
    {
        concurrency::LockGuard guard(&childSeenLock);
        childLastSeenMs[from] = nowMs;
        // forget children silent for > 1 h so the map stays small
        for (auto it = childLastSeenMs.begin(); it != childLastSeenMs.end();) {
            if ((uint32_t)(nowMs - it->second) > 3600000UL)
                it = childLastSeenMs.erase(it);
            else
                ++it;
        }
    }
    uint32_t msSinceChildSeen(NodeNum n) const
    {
        concurrency::LockGuard guard(&childSeenLock);
        auto it = childLastSeenMs.find(n);
        if (it == childLastSeenMs.end())
            return UINT32_MAX;
        return (uint32_t)(millis() - it->second);
    }

    uint32_t lastPanicEventId = 0;
    uint32_t nextEventId = 1;
    uint32_t lastLocateReqAt = 0;

    // Explicit panic state machine (ARCH §2, BUG-008/012/013).
    FamilyPanicState panicState = FamilyPanicState::NORMAL;
    uint32_t lastPanicSentMs = 0;

    // Startup sync (BUG-011/ENH-007): child CHECKINs immediately on first runOnce
    // (not after a full interval); parent broadcasts PARENT_PRESENCE once. A one-
    // shot NodeInfo burst also runs at startup so phones see a fresh timestamp.
    // If a GPS fix lands within 2 min of boot, one immediate position update
    // follows so the family gets the location quickly instead of waiting a full
    // broadcast interval.
    uint32_t lastCheckinMs = 0;
    bool startupCheckinPending = true;
    bool startupPresencePending = true;
    bool startupNodeInfoSent = false;
    bool quickFixPending = true;
    uint32_t bootAtMs = 0;
    uint32_t lastPresenceMs = 0;

    // Watchdog tuning (remote CONFIG message, FAMILYTRACKER_MSG_CONFIG).
    uint32_t missedTimeoutSecs = FAMILYTRACKER_DEFAULT_MISSED_TIMEOUT_SECS;
    uint8_t lowBatteryPct = FAMILYTRACKER_DEFAULT_LOW_BATTERY_PCT;

    // Most recent panicking child (parents): target for a quick "on my way"
    // response when the parent doesn't name one.
    NodeNum lastPanicChild = 0;
    uint32_t lastPanicChildMs = 0;

    // Child-side mirror of the parent watchdog: if NO parent has been heard for
    // missedTimeoutSecs, the child alerts locally (come-back tone + banner).
    // Requires prior contact - a freshly booted child never false-alarms.
    void markParentHeard();
    uint32_t lastParentHeardMs = 0;
    bool parentHeardEver = false;
    bool parentMissingAlerted = false;

    // Missed-check-in spam control: only ONE parent reports each child's missed
    // check-in to the group. The reporter sets a suppression window; other
    // parents suppress on hearing the reporter's MISSED_ALERT datagram.
    std::map<NodeNum, uint32_t> missedSuppressedUntilMs;

    // Parent-triggered flows (COME_BACK / LOST_CHILD / FIND_SOUND).
    bool lostModeActive = false;    // child: fast check-in cadence until a parent sends FOUND
    uint32_t findSoundUntilMs = 0;  // any role: re-beep the find tone until this time
    uint32_t lastFindBeepMs = 0;
    uint32_t lastComeBackRxMs = 0;  // child: dedup double come-back tunes (both parents fire)

    // A full panic (banner + buzzer + ACK) is queued here and acted on in
    // runOnce, so NO side-effect runs inside the radio RX handler. Doing banner/
    // buzzer/TX inside handleReceived contends with the radio/flash SPI lock on
    // the nRF52 and intermittently hangs the T1 parent.
    // Ring buffer (BUG: single slot dropped child #2's alarm when two kids
    // panicked within one runOnce tick). Guarded by stateLock.
    struct PendingPanic {
        bool active = false;
        NodeNum from = 0;
        uint32_t eventId = 0;
        uint32_t eventTs = 0;
        meshtastic_PositionLite pos;
        bool stale = false;
        uint8_t ageMin = 0;
        bool hasPos = false;
    };
    static const uint8_t PENDING_PANIC_LEN = 6;
    PendingPanic pendingPanics[PENDING_PANIC_LEN];
    uint8_t pendingPanicHead = 0;   // next slot to WRITE
    uint8_t pendingPanicCount = 0;  // waiting to render

    // Serialises the RX-thread writers (handleReceived/handleTextCommand)
    // against this module's OSThread reader (runOnce watchdog/drain) for:
    // pendingPanics ring, lastPanicEventIdByNode, lastPanicAlertMsByNode,
    // missedSuppressedUntilMs, alertQueue and lastPanicChild. std::map
    // insert/find from two threads without a lock corrupts the heap - the
    // exact failure class that got childLastSeenMs its own lock earlier.
    mutable concurrency::Lock stateLock;

    // Deferred human-readable text: queueTextAlert() from the RX path must not
    // send directly (the reentrant flash save hangs the T1); flushed by runOnce.
    // BUG-017: ring buffer, not a single slot - rapid events (panic + regroup +
    // found) were overwriting each other before the flush, dropping messages.
    // head/count are guarded by stateLock.
    void queueTextAlert(const char *format, ...);
    static const uint8_t ALERT_QUEUE_LEN = 6;
    char alertQueue[ALERT_QUEUE_LEN][128] = {{0}};
    uint8_t alertQueueHead = 0;  // next slot to WRITE
    uint8_t alertQueueCount = 0; // slots waiting

    // ENH-009 developer debug mode (build-time FAMILY_DEBUG_CHAT): echo tracker
    // protocol events into family chat with a "[FT] " prefix. Compiled out of
    // release builds entirely.
    void ftDebug(const char *format, ...);

    // ENH-008: persistent panic indication - SOS pattern on PIN_LED1 while a
    // panic is active on this child (+ repeating banner on screen hardware).
    // Cleared when the panic stands down.
    uint32_t lastPanicIndicateMs = 0;
    bool panicLedActive = false;

    // Child tracker default view (BUG-010/ENH-010): the nearest-parent arrow is
    // now a dedicated "Parent" screen frame (Screen::setFrames). This retained
    // as a PARENT_PRESENCE warm-up + log hook (no more banner overlay).
    void updateNearestParentDisplay();
    uint32_t lastNearestParentMs = 0;

    // Auto-favourite family members (Friend-Finder/ENH): every node that sends a
    // valid family datagram is favourited once. set_favorite() saves the node DB
    // to flash, so the writes are deferred to runOnce (never inside the radio RX
    // handler, which must not touch flash/SPI). autoFavourite() runs on the
    // Router thread, runOnce on this module's OSThread.
    void autoFavourite(NodeNum num);
    std::vector<NodeNum> pendingFavourites;
    std::vector<NodeNum> favouriteQueued;
    mutable concurrency::Lock favouriteLock;

    // Remote-admin trust (family default): copy the public keys of PARENT-role
    // family nodes into config.security.admin_key[0..2], so any parent can PKI-
    // admin any other family node out of the box. Retried hourly-ish early on -
    // the nodedb (and its public keys) fills as peers announce.
    void syncAdminTrust();
    uint8_t trustSyncAttempts = 0;
    uint32_t lastTrustSyncMs = 0;

    // Stationary base/relay mobility guard: a base-role node (camp / car) that
    // moves > FAMILYTRACKER_BASE_MOVE_METRES from its boot anchor has been picked
    // up by a searching adult - promote it to a full CLIENT parent (persisted +
    // NodeInfo rebroadcast) so it joins alerts AND becomes a nearest-parent
    // navigation target on the children's screens.
    void updateBaseMobility();
    bool baseAnchorSet = false;
    int32_t baseAnchorLat = 0;
    int32_t baseAnchorLon = 0;
    bool mobilePromoted = false;
};

extern FamilyTrackerModule *familyTrackerModule;

// LTO guard accessor (see FamilyTrackerModule.cpp)
__attribute__((used)) FamilyTrackerModule *getFamilyTrackerModule();
