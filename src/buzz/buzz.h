#pragma once

void playBeep();
void playLongBeep();
void playStartMelody();
void playShutdownMelody();
void playGPSEnableBeep();
void playGPSDisableBeep();
void playComboTune();
void play4ClickDown();
void play4ClickUp();
void playBoop();
void playChirp();
void playClick();
bool playNextLeadUpNote();  // Play the next note in the lead-up sequence
void resetLeadUpSequence(); // Reset the lead-up sequence to start from beginning

// Family Tracker tones (distinct per event).
void playPanicCall();     // child: panic SENT - three short "call" notes
void playPanicAlert();    // parent: panic RECEIVED - urgent two-tone alert
void playPanicResponse(); // child: panic ACKED - rising "response" completing the call/response
void playSosTone();       // child: lost - SOS in morse (... --- ...)
void playLostAlert();     // parent: another child reported lost - distinct alert
void playMarioMelody();   // child: come-back - super-mario-style tune
void playFoundMelody();   // all: "found / level complete" success fanfare
void playMissedCheckinTone(); // parent: child missed a check-in - distinct double-warning
void playLowBatteryTone();    // parent: child battery low - distinct double-beep