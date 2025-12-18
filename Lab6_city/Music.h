#ifndef MUSIC_H
#define MUSIC_H

#include <Arduino.h>
#include "protothreads.h"

class Music {
public:
    Music(uint8_t pin);

    void begin();
    PT_THREAD(play(struct pt *pt));
    void stop();

protected:
    const int *melody;
    int notes;
    int tempo;
    int wholenote;

    uint8_t buzzer;

    // protothread state
    int thisNote;
    int divider;
    int noteDuration;
    unsigned long timer;
};

#endif