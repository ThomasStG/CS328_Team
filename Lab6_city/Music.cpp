#include <Arduino.h>
#include "Music.h"

Music::Music(uint8_t buzzerPin)
    : melody(nullptr),
      notes(0),
      tempo(120),
      wholenote(0),
      buzzer(buzzerPin),
      thisNote(0),
      divider(0),
      noteDuration(0),
      timer(0) {}

void Music::begin() {
    pinMode(buzzer, OUTPUT);
    wholenote = (60000 * 4) / tempo;
    thisNote = 0;
}

void Music::stop() {
    noTone(buzzer);
    thisNote = 0;
}

PT_THREAD(Music::play(struct pt *pt)) {
    
    PT_BEGIN(pt);

    while (true) { 
        thisNote = 0; 
        while (thisNote < notes * 2) {
            divider = pgm_read_word(&melody[thisNote + 1]);

            if (divider > 0) {
                noteDuration = wholenote / divider;
            } else {
                noteDuration = wholenote / abs(divider);
                noteDuration *= 1.5;
            }

            int freq = pgm_read_word(&melody[thisNote]);
            if (freq != 0) {
                tone(buzzer, freq, noteDuration * 0.9);
            }

            timer = millis() + noteDuration;
            while (millis() < timer) {
                PT_YIELD(pt);
            }

            noTone(buzzer);
            thisNote += 2;
        }
    }

    PT_END(pt);
}