/**************************************************************************
 * Name:    Timothy Lamb                                                  *
 * Email:   trash80@gmail.com                                             *
 ***************************************************************************/
/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#define MGB_MIDI_DELAY        350 // µs (test to see how low delay mGB can handle) 
#define VIBR_UPDATE_INTERVAL   10 // ms
#define VIBR_RATE_MAX         500 // ms
#define PB_CENTER            8192 // Pitch bend center value (0..16383)
#define CC_VIBRATO_RATE         6
#define CC_VIBRATO_DEPTH        7

int16_t pbCurrent[3] = {PB_CENTER,PB_CENTER,PB_CENTER}; // Keep track of current pitch bend value
int16_t pbSend[3];                   // Used temporarily when combining pitch bend and vibrato to be sent as pitch bend to mGB
int16_t vibrDepth[3] = {0,0,0};      // The depth of the vibrato effect, can be changed with CC=7
int16_t vibrRate[3] = {200,200,200}; // The rate of the vibrato effect, can be changed with CC=6
float   vibr[3] = {0,0,0};           // Calculated vibrato
int16_t vibrInt[3] = {0,0,0};        // Calculated vibrato converted to an integer
int32_t millisNow;                   // Keep track of time
int32_t millisUpdate[3];             // Keep track of timestamp for last vibrato update

uint8_t midiVibratoDepthMode = false;
uint8_t midiVibratoRateMode = false;

void modeMidiGbSetup()
{
  digitalWrite(pinStatusLed,LOW);
  pinMode(pinGBClock,OUTPUT);
  digitalWrite(pinGBClock,HIGH);

#ifdef USE_TEENSY
  usbMIDI.setHandleRealTimeSystem(NULL);
#endif

  blinkMaxCount=1000;
  modeMidiGb();
}

void modeMidiGb()
{
  uint8_t ch;
  boolean sendByte = false;
  while(1){                                //Loop foreverrrr
    millisNow = millis();
    modeMidiGbUsbMidiReceive();

    // Handle pitch modulation (not in the middle of a MIDI command)
    if(!midiValueMode)
    {
      for(ch=0; ch<3; ch++)
      {
        if(vibrDepth[ch] > 0 && (millisNow - millisUpdate[ch]) > VIBR_UPDATE_INTERVAL)
        {  
          vibr[ch] = (float)(vibrDepth[ch]/vibrRate[ch]) * (vibrRate[ch] - abs(millisNow % (2*vibrRate[ch]) - vibrRate[ch])) - (vibrDepth[ch]/2);
          vibrInt[ch] = (int)vibr[ch];
          pbSend[ch] = constrain(pbCurrent[ch]+vibrInt[ch],0,16383);
          sendByteToGameboy(0xE0+ch); // TODO: Don't send value if in the middle of another MIDI message!!!!
          delayMicroseconds(MGB_MIDI_DELAY);
          sendByteToGameboy(pbSend[ch] & 0x7F);
          delayMicroseconds(MGB_MIDI_DELAY);
          sendByteToGameboy(pbSend[ch] >> 7);
          delayMicroseconds(MGB_MIDI_DELAY);
          millisUpdate[ch] = millisNow;
        }
      }
    }

    if (serial->available()) {          //If MIDI is sending
      incomingMidiByte = serial->read();    //Get the byte sent from MIDI

      if(!checkForProgrammerSysex(incomingMidiByte) && !usbMode) serial->write(incomingMidiByte); //Echo the Byte to MIDI Output

      if(incomingMidiByte & 0x80) { // MIDI status byte (message type)
        switch (incomingMidiByte & 0xF0) {
          case 0xA0: // Polyphonic Aftertouch
          case 0xD0: // Channel Aftertouch
          case 0xF0: // SysEx
            midiValueMode = false;
            break;
          default:
            sendByte = false;
            midiStatusChannel = incomingMidiByte&0x0F;
            midiStatusType    = incomingMidiByte&0xF0;
            if(midiStatusChannel == memory[MEM_MGB_CH]) {
               midiData[0] = midiStatusType;
               sendByte = true;
            } else if (midiStatusChannel == memory[MEM_MGB_CH+1]) {
               midiData[0] = midiStatusType+1;
               sendByte = true;
            } else if (midiStatusChannel == memory[MEM_MGB_CH+2]) {
               midiData[0] = midiStatusType+2;
               sendByte = true;
            } else if (midiStatusChannel == memory[MEM_MGB_CH+3]) {
               midiData[0] = midiStatusType+3;
               sendByte = true;
            } else if (midiStatusChannel == memory[MEM_MGB_CH+4]) {
               midiData[0] = midiStatusType+4;
               sendByte = true;
            } else {
              midiValueMode  =false;
              midiAddressMode=false;
            }
            if(sendByte) {
              statusLedOn();
              midiValueMode  =false;
              midiAddressMode=true;
            }
           break;
        }
      } 
      else if (midiAddressMode)
      {
        midiAddressMode = false;
        midiData[1] = incomingMidiByte;
        switch(midiStatusType)
        {
          case 0xE0: // Pitch Bend
            sendByteToGameboy(midiData[0]);
            delayMicroseconds(MGB_MIDI_DELAY);
            midiValueMode = true;
            break;

          case 0xB0: // Control Change
            if( (incomingMidiByte > 0 && incomingMidiByte < 6) || incomingMidiByte == 10 || incomingMidiByte == 64) // Just send CCs that mGB uses
            { 
              midiValueMode = true;
              sendByteToGameboy(midiData[0]);
              delayMicroseconds(MGB_MIDI_DELAY);
              sendByteToGameboy(midiData[1]);
              delayMicroseconds(MGB_MIDI_DELAY);
            }
            else if(incomingMidiByte == CC_VIBRATO_DEPTH)     // Set vibrato depth
            {
              midiVibratoDepthMode = true;
            }
            else if(incomingMidiByte == CC_VIBRATO_RATE)  // Set vibrato rate
            {
              midiVibratoRateMode = true;
            }
            break;

          default:
            midiValueMode = true;
            sendByteToGameboy(midiData[0]);
            delayMicroseconds(MGB_MIDI_DELAY);
            sendByteToGameboy(midiData[1]);
            delayMicroseconds(MGB_MIDI_DELAY);
            break;
        }
      } 
      else if (midiValueMode) 
      {
        midiData[2] = incomingMidiByte;
        midiAddressMode = true;
        midiValueMode = false;
        switch(midiStatusType)
        {
          case 0xE0: // Pitch Bend
            pbCurrent[midiStatusChannel] = midiData[1] | (midiData[2] << 7);
            pbSend[midiStatusChannel] = constrain(pbCurrent[midiStatusChannel]+vibrInt[midiStatusChannel],0,16383);
            sendByteToGameboy(pbSend[midiStatusChannel] & 0x7F);
            delayMicroseconds(MGB_MIDI_DELAY);
            sendByteToGameboy(pbSend[midiStatusChannel] >> 7);
            delayMicroseconds(MGB_MIDI_DELAY);
            break;

          default:
            sendByteToGameboy(midiData[2]);
            delayMicroseconds(MGB_MIDI_DELAY);
            break;
        }
        statusLedOn();
        blinkLight(midiData[0],midiData[2]);
      }
      else if(midiVibratoDepthMode)
      {
        midiData[2] = incomingMidiByte;
        midiAddressMode = true;
        midiVibratoDepthMode = false;
        vibrDepth[midiStatusChannel] = map(midiData[2], 0, 127, 0, 16383);
        if(vibrDepth[midiStatusChannel] == 0) // Reset pitch bend to current if vibrato depth becomes 0
        {
          vibrInt[midiStatusChannel] = 0; // Reset vibrato value
          sendByteToGameboy(0xE0 + midiStatusChannel);
          delayMicroseconds(MGB_MIDI_DELAY);
          sendByteToGameboy(pbCurrent[midiStatusChannel] & 0x7F);
          delayMicroseconds(MGB_MIDI_DELAY);
          sendByteToGameboy(pbCurrent[midiStatusChannel] >> 7);
          delayMicroseconds(MGB_MIDI_DELAY);
        }
      }
      else if(midiVibratoRateMode)
      {
        midiData[2] = incomingMidiByte;
        vibrRate[midiStatusChannel] = map(midiData[2], 0, 127, 50, VIBR_RATE_MAX);
        midiAddressMode = true;
        midiVibratoRateMode = false;   
      }
    } else {
      setMode();                // Check if mode button was depressed
      updateBlinkLights();
      updateStatusLed();
    }
  }
}

 /*
 sendByteToGameboy does what it says. yay magic
 */
void sendByteToGameboy(byte send_byte)
{
 for(countLSDJTicks=0;countLSDJTicks!=8;countLSDJTicks++) {  //we are going to send 8 bits, so do a loop 8 times
   if(send_byte & 0x80) {
       GB_SET(0,1,0);
       GB_SET(1,1,0);
   } else {
       GB_SET(0,0,0);
       GB_SET(1,0,0);
   }

#if defined (F_CPU) && (F_CPU > 24000000)
   // Delays for Teensy etc where CPU speed might be clocked too fast for cable & shift register on gameboy.
   delayMicroseconds(1);
#endif
   send_byte <<= 1;
 }
}

void modeMidiGbUsbMidiReceive()
{
#ifdef USE_TEENSY

    while(usbMIDI.read()) {
        uint8_t ch = usbMIDI.getChannel() - 1;
        boolean send = false;
        if(ch == memory[MEM_MGB_CH]) {
            ch = 0;
            send = true;
        } else if (ch == memory[MEM_MGB_CH+1]) {
            ch = 1;
            send = true;
        } else if (ch == memory[MEM_MGB_CH+2]) {
            ch = 2;
            send = true;
        } else if (ch == memory[MEM_MGB_CH+3]) {
            ch = 3;
            send = true;
        } else if (ch == memory[MEM_MGB_CH+4]) {
            ch = 4;
            send = true;
        }
        if(!send) return;
        uint8_t s;
        switch(usbMIDI.getType()) {
            case 0x80: // note off
            case 0x90: // note on
                s = 0x90 + ch;
                if(usbMIDI.getType() == 0x80) {
                    s = 0x80 + ch;
                }
                sendByteToGameboy(s);
                delayMicroseconds(MGB_MIDI_DELAY);
                sendByteToGameboy(usbMIDI.getData1());
                delayMicroseconds(MGB_MIDI_DELAY);
                sendByteToGameboy(usbMIDI.getData2());
                delayMicroseconds(MGB_MIDI_DELAY);
                blinkLight(s, usbMIDI.getData2());
            break;
            case 0xB0: // CC
                sendByteToGameboy(0xB0+ch);
                delayMicroseconds(MGB_MIDI_DELAY);
                sendByteToGameboy(usbMIDI.getData1());
                delayMicroseconds(MGB_MIDI_DELAY);
                sendByteToGameboy(usbMIDI.getData2());
                delayMicroseconds(MGB_MIDI_DELAY);
                blinkLight(0xB0+ch, usbMIDI.getData2());
            break;
            case 0xC0: // PG
                sendByteToGameboy(0xC0+ch);
                delayMicroseconds(MGB_MIDI_DELAY);
                sendByteToGameboy(usbMIDI.getData1());
                delayMicroseconds(MGB_MIDI_DELAY);
                blinkLight(0xC0+ch, usbMIDI.getData2());
            break;
            case 0xE0: // PB
                sendByteToGameboy(0xE0+ch);
                delayMicroseconds(MGB_MIDI_DELAY);
                sendByteToGameboy(usbMIDI.getData1());
                delayMicroseconds(MGB_MIDI_DELAY);
                sendByteToGameboy(usbMIDI.getData2());
                delayMicroseconds(MGB_MIDI_DELAY);
            break;
        }

        statusLedOn();
    }
#endif

#ifdef USE_LEONARDO

    midiEventPacket_t rx;
      do
      {
        rx = MidiUSB.read();
        uint8_t ch = rx.byte1 & 0x0F;
        boolean send = false;
        if(ch == memory[MEM_MGB_CH]) {
            ch = 0;
            send = true;
        } else if (ch == memory[MEM_MGB_CH+1]) {
            ch = 1;
            send = true;
        } else if (ch == memory[MEM_MGB_CH+2]) {
            ch = 2;
            send = true;
        } else if (ch == memory[MEM_MGB_CH+3]) {
            ch = 3;
            send = true;
        } else if (ch == memory[MEM_MGB_CH+4]) {
            ch = 4;
            send = true;
        }
        if (!send) return;
        uint8_t s;
        switch (rx.header)
        {
        case 0x08: // note off
        case 0x09: // note on
          s = 0x90 + ch;
          if (rx.header == 0x08)
          {
            s = 0x80 + ch;
          }
          sendByteToGameboy(s);
          delayMicroseconds(MGB_MIDI_DELAY);
          sendByteToGameboy(rx.byte2);
          delayMicroseconds(MGB_MIDI_DELAY);
          sendByteToGameboy(rx.byte3);
          delayMicroseconds(MGB_MIDI_DELAY);
          blinkLight(s, rx.byte2);
          break;
        case 0x0B: // Control Change
          if( (rx.byte2 > 0 && rx.byte2 < 6) || rx.byte2 == 10 || rx.byte2 == 64) // Just send CCs that mGB uses
          { 
            sendByteToGameboy(0xB0 + ch);
            delayMicroseconds(MGB_MIDI_DELAY);
            sendByteToGameboy(rx.byte2);
            delayMicroseconds(MGB_MIDI_DELAY);
            sendByteToGameboy(rx.byte3);
            delayMicroseconds(MGB_MIDI_DELAY);
            blinkLight(0xB0 + ch, rx.byte2);
          }
          else 
          {
            if(rx.byte2 == CC_VIBRATO_DEPTH) {     // Set vibrato depth
              vibrDepth[ch] = map(rx.byte3, 0, 127, 0, 16383);
              if(vibrDepth[ch] == 0) // Reset pitch bend to current if vibrato depth becomes 0
              {
                vibrInt[ch] = 0; // Reset vibrato value
                sendByteToGameboy(0xE0 + ch);
                delayMicroseconds(MGB_MIDI_DELAY);
                sendByteToGameboy(pbCurrent[ch] & 0x7F);
                delayMicroseconds(MGB_MIDI_DELAY);
                sendByteToGameboy(pbCurrent[ch] >> 7);
                delayMicroseconds(MGB_MIDI_DELAY);
              }
            }
            else if(rx.byte2 == CC_VIBRATO_RATE) // Set vibrato rate
              vibrRate[ch] = map(rx.byte3, 0, 127, 50, VIBR_RATE_MAX);
          }
          break;
        case 0x0C: // Program Change
          sendByteToGameboy(0xC0 + ch);
          delayMicroseconds(MGB_MIDI_DELAY);
          sendByteToGameboy(rx.byte2);
          delayMicroseconds(MGB_MIDI_DELAY);
          blinkLight(0xC0 + ch, rx.byte2);
          break;
        case 0x0E: // Pitch Bend
          pbCurrent[ch] = rx.byte2 | (rx.byte3 << 7);
          pbSend[ch] = constrain(pbCurrent[ch]+vibrInt[ch],0,16383);
          sendByteToGameboy(0xE0 + ch);
          delayMicroseconds(MGB_MIDI_DELAY);
          sendByteToGameboy(pbSend[ch] & 0x7F);
          delayMicroseconds(MGB_MIDI_DELAY);
          sendByteToGameboy(pbSend[ch] >> 7);
          delayMicroseconds(MGB_MIDI_DELAY);
          break;
        default:
          return;
        }

        statusLedOn();
      } while (rx.header != 0);
#endif
}
