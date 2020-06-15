#include <Arduino.h>

enum class ParserMode
{
    kWaitingForStartCharacter,
    kReading,
};

struct Parser
{
    static const uint8_t kBufferSize = 128;
    uint8_t buffer[kBufferSize];
    uint8_t buffer_write_index;
};

void UpdateParser(Parser &parser)
{
    if (Serial.available())
    {
        char c = Serial.read()
        //
        //
        if waiting for start marker:
            if c i sthe start marker
                change our state to reading mode
            else
            {
                dont do anything
            }
        elif reading mode
            if c is the end marker
                return the buffer
            else
                add c to the buffer
            end
        end
    }
}

/** 
 * in the main program
 * loop()
 * {
 *      updateParser()
 * }