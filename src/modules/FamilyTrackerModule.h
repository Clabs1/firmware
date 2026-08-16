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
 * Payload v0.2 (max 21 bytes):
 *   byte 0      : protocol version (0x02 = v0.2, stays < v1)
 *   byte 1      : msg type (0x01 PANIC, 0x02 PANIC_ACK, 0x03 LOCATE_REQ, 0x04 LOCATE_RESP)
 *   bytes 2-5   : event id (uint32 LE)
 *   byte 6      : flags (bit0 HAS_POS, bit1 POS_STALE)
 *   byte 7      : pos_age_minutes
 *   bytes 8-11  : latitude_i (int32 LE)   [if HAS_POS]
 *   bytes 12-15 : longitude_i (int32 LE)  [if HAS_POS]
 *   bytes 16-17 : altitude (int16 LE)     [if HAS_POS]
 *   bytes 18-19 : battery (uint16 LE)     [if available]
 */
#define FAMILYTRACKER_PROTOCOL_VERSION 0x02

#define FAMILYTRACKER_MSG_PANIC       0x01
#define FAMILYTRACKER_MSG_PANIC_ACK   0x02
#define FAMILYTRACKER_MSG_LOCATE_REQ  0x03
#define FAMILYTRACKER_MSG_LOCATE_RESP 0x04
#define FAMILYTRACKER_MSG_CHECKIN     0x05
#define FAMILYTRACKER_MSG_PANIC_TRIGGER 0x06
#define FAMILYTRACKER_MSG_CONFIG      0x07  // set watchdog timeout / low-battery % (remote test config)
#define FAMILYTRACKER_MSG_PARENT_ON_WAY  0x08  // parent -> child: "read + on the way" (preselected msg)
#define FAMILYTRACKER_MSG_ON_WAY_TRIGGER 0x09  // remote trigger for hands-off testing (like PANIC_TRIGGER)

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

// Child periodic check-in cadence (SPEC §13/§14/§18) - independent of GPS so the
// parent can always distinguish "idle" from "missing".
#define FAMILYTRACKER_CHECKIN_INTERVAL_SECS 120

class FamilyTrackerModule : public SinglePortModule, public concurrency::OSThread
{
  public:
    // __attribute__((used)) prevents whole-image LTO (nrf52_lto.py) from treating the
    // constructor call in setupModules() as dead code, since the result is discarded.
    __attribute__((used)) FamilyTrackerModule();

    bool wantPacket(const meshtastic_MeshPacket *p) override
    {
        return p->decoded.portnum == meshtastic_PortNum_PRIVATE_APP;
    }

    ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

    void setup() override;

    int handleInputEvent(const InputEvent *event);

    int32_t runOnce() override; // OSThread: parent watchdog tick

    CallbackObserver<FamilyTrackerModule, const InputEvent *> inputObserver =
        CallbackObserver<FamilyTrackerModule, const InputEvent *>(this, &FamilyTrackerModule::handleInputEvent);

  private:
    bool isChild() const;
    bool isParent() const;

    void sendMessage(uint8_t msgType, uint32_t eventId, bool hasPos, bool stale, uint8_t ageMin);
    void sendMessageTo(uint8_t msgType, uint32_t eventId, bool hasPos, bool stale, uint8_t ageMin, NodeNum to);
    void sendTextAlert(const char *format, ...);
    // sendAck: acknowledge the panic to the specific child that raised it
    void sendAck(uint32_t eventId, NodeNum to);

    void buzzerBeep(bool ack);
    bool isValidMessage(const meshtastic_MeshPacket &mp, uint8_t *msgType, uint32_t *eventId, uint8_t *flags,
                        uint8_t *ageMin, meshtastic_PositionLite *pos);
    void fillBestPosition(meshtastic_PositionLite *pos, bool *hasPos, bool *stale, uint8_t *ageMin);
    void triggerFreshFix();

    // Human-readable panic rendering (SPEC §34 "family channel human-readable")
    // returns "<name> pressed the panic button at hh:mm - <dist> <dir> (<age>)"
    void renderPanicAlert(const meshtastic_MeshPacket &mp, uint32_t eventId, const meshtastic_PositionLite &pos,
                          bool stale, uint8_t ageMin, bool hasPos);

    // Send the preselected "on the way" response to a specific child: a Family
    // Channel text (human-readable) + PARENT_ON_WAY datagram (distinct tone +
    // on-screen message at the child). SPEC §34A.
    void sendOnWay(uint32_t eventId, uint8_t presetIndex, NodeNum to);

    // Parent watchdog state
    std::vector<NodeNum> alertedMissed;
    std::vector<NodeNum> alertedBattery;
    std::vector<NodeNum> alertedPanic;   // dedup: children already alerted for

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

    // Watchdog tuning (remote CONFIG message, FAMILYTRACKER_MSG_CONFIG).
    uint32_t missedTimeoutSecs = FAMILYTRACKER_DEFAULT_MISSED_TIMEOUT_SECS;
    uint8_t lowBatteryPct = FAMILYTRACKER_DEFAULT_LOW_BATTERY_PCT;
};

extern FamilyTrackerModule *familyTrackerModule;

// LTO guard accessor (see FamilyTrackerModule.cpp)
__attribute__((used)) FamilyTrackerModule *getFamilyTrackerModule();
