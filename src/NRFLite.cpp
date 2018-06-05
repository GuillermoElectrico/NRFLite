#include <NRFLite.h>

#define debug(input)   { if (_serial) _serial->print(input);   }
#define debugln(input) { if (_serial) _serial->println(input); }

#if defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    const static uint8_t USI_DI  = 6; // PA6
    const static uint8_t USI_DO  = 5; // PA5
    const static uint8_t USI_SCK = 4; // PA4
#elif defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    const static uint8_t USI_DI  = 0; // PB0
    const static uint8_t USI_DO  = 1; // PB1
    const static uint8_t USI_SCK = 2; // PB2
#else
    #include <SPI.h> // Use the normal Arduino hardware SPI library.
#endif

////////////////////
// Public methods //
////////////////////

uint8_t NRFLite::init(uint8_t radioId, uint8_t cePin, uint8_t csnPin, Bitrates bitrate, uint8_t channel)
{
    _useTwoPinSpiTransfer = 0;
    _cePin = cePin;
    _csnPin = csnPin;
    
    // Default states for the radio pins.  When CSN is LOW the radio listens to SPI communication,
    // so we operate most of the time with CSN HIGH.
    pinMode(_cePin, OUTPUT);
    pinMode(_csnPin, OUTPUT);
    digitalWrite(_csnPin, HIGH);
    
    // Setup the microcontroller for SPI communication with the radio.
    #if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        pinMode(USI_DI, INPUT ); digitalWrite(USI_DI, HIGH);
        pinMode(USI_DO, OUTPUT); digitalWrite(USI_DO, LOW);
        pinMode(USI_SCK, OUTPUT); digitalWrite(USI_SCK, LOW);
    #else
        // Arduino SPI makes SS (D10) an output and sets it HIGH.  It must remain an output
        // for Master SPI operation to work, but in case it was originally LOW, we'll set it back.
        uint8_t savedSS = digitalRead(SS);
        SPI.setClockDivider(SPI_CLOCK_DIV2);
        SPI.begin();
        if (_csnPin != SS) digitalWrite(SS, savedSS);
    #endif
    
    return prepForRx(radioId, bitrate, channel);
}

#if defined(__AVR__)

uint8_t NRFLite::initTwoPin(uint8_t radioId, uint8_t momiPin, uint8_t sckPin, Bitrates bitrate, uint8_t channel)
{
    _useTwoPinSpiTransfer = 1;
    _cePin = sckPin;
    _csnPin = sckPin;

    // Default states for the 2 multiplexed pins.
    pinMode(momiPin, INPUT);
    pinMode(sckPin, OUTPUT); digitalWrite(sckPin, HIGH);

    // These port and mask functions are in Arduino.h, e.g. arduino-1.8.1\hardware\arduino\avr\cores\arduino\Arduino.h
    // Direct port manipulation is used since timing is critical for the multiplexed SPI bit-banging.
    _momi_PORT = portOutputRegister(digitalPinToPort(momiPin));
    _momi_DDR = portModeRegister(digitalPinToPort(momiPin));
    _momi_PIN = portInputRegister(digitalPinToPort(momiPin));
    _momi_MASK = digitalPinToBitMask(momiPin);
    _sck_PORT = portOutputRegister(digitalPinToPort(sckPin));
    _sck_MASK = digitalPinToBitMask(sckPin);

    return prepForRx(radioId, bitrate, channel);
}

#endif

void NRFLite::addAckData(void *data, uint8_t length)
{
    if (_useSack)
    {
        if (length > 30) length = 30; // Limit user's data to 30 bytes.
        
        // Store data in the byte array used when transmitting SACK packets.
        _sackData[0] = _radioId;
        _sackData[1] = _lastSackDataId++;
        _sackDataLength = length + 2;

        uint8_t* intData = reinterpret_cast<uint8_t*>(data);
        for (uint8_t i = 0; i < length; i++)
        {
            _sackData[i + 2] = intData[i];
        }
    }
    else
    {
        // Ensure any previously inserted ACKs are removed.  The hardware supports 3 of these packets but
        // this isn't implemented in the library.
        removeAckData();

        // Add the packet to the TX FIFO buffer for pipe 1, the pipe used to receive packets from radios that
        // send us data.  When we receive the next transmission from a radio, we'll provide this ACK data in the
        // auto-acknowledgment packet that goes back.
        spiTransfer(WRITE_OPERATION, (W_ACK_PAYLOAD | 1), data, length);
    }
}

void NRFLite::removeAckData()
{
    if (_useSack)
    {
        _sackDataLength = 0;
    }
    else
    {
        spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0); // Clear any previously inserted ACK data packets.
    }
}

uint8_t NRFLite::hasAckData()
{
    if (_sackDataLength > 0)
    {
        return _sackDataLength - 2; // Exclude the 2 byte packet id.
    }
    else if (getPipeOfFirstRxPacket() == 0) // Pipe 0 receives hardware-based ACK packets.
    {
        return getRxPacketLength();
    }
    else
    {
        return 0;
    }
}

uint8_t NRFLite::hasData(uint8_t usingInterrupts)
{
    // If using the same pins for CE and CSN, we need to ensure CE is left HIGH long enough to receive data.
    // If we don't limit the calling program, CE may mainly be LOW and the radio won't get a chance
    // to receive packets.  However, if the calling program is using an interrupt handler and only calling
    // hasData when the data received flag is set, we should skip this check since we know the calling program
    // is not continually polling hasData.  So 'usingInterrupts' = 1 bypasses the logic.
    if (_cePin == _csnPin && !usingInterrupts)
    {
        if (micros() - _microsSinceLastDataCheck < _maxHasDataIntervalMicros)
        {
            return 0; // Prevent the calling program from forcing us to bring CE low, making the radio stop receiving.
        }
        else
        {
            _microsSinceLastDataCheck = micros();
        }
    }
    
    // Ensure radio is powered on and in RX mode in case the radio was powered down or in TX mode.
    uint8_t configReg = readRegister(CONFIG);
    uint8_t newConfigReg = configReg | _BV(PWR_UP) | _BV(PRIM_RX);
    if (configReg != newConfigReg) 
    { 
        writeRegister(CONFIG, newConfigReg); 
    }
    
    // Ensure we're listening for packets by setting CE HIGH.  If we share the same pin for CE and CSN,
    // it will already be HIGH since we always keep CSN HIGH to prevent the radio from listening to the SPI bus.
    if (_cePin != _csnPin)
    { 
        if (digitalRead(_cePin) == LOW)
        {
            digitalWrite(_cePin, HIGH);
            delayMicroseconds(STANDBY_TO_RXTX_MODE_MICROS);
        }
    }
    
    // If the radio was initially powered off, wait for it to turn on.
    if ((configReg & _BV(PWR_UP)) == 0)
    { 
        delayMicroseconds(POWERDOWN_TO_RXTX_MODE_MICROS);
    }

    uint8_t pipe = getPipeOfFirstRxPacket();

    if (pipe == 1) // Pipe 1 receives REQUIRE_ACK and NO_ACK packets.
    {
        _useSack = 0;
        return getRxPacketLength();
    }
    else if (pipe == 2) // Pipe 2 receives REQUIRE_SACK packets (software-based ACK packets).
    {
        _useSack = 1;
        return getSackDataLengthAndSendAck();
    }
    else
    {
        return 0;
    }
}

uint8_t NRFLite::hasDataISR()
{
    // This method can be used inside an interrupt handler for the radio's IRQ pin to bypass
    // the limit on how often the radio can be checked for data.  This optimization greatly increases
    // the receiving bitrate when CE and CSN share the same pin.
    return hasData(1); // usingInterrupts = 1
}

void NRFLite::readData(void *data)
{
    if (_receivedDataLength > 0)
    {
        // Receiver received a REQUIRE_SACK packet and is reading it.
        uint8_t* intData = reinterpret_cast<uint8_t*>(data);
        for (uint8_t i = 0; i < _receivedDataLength - 2; i++)
        {
            intData[i] = _requireSackData[i + 2];
        }
        _receivedDataLength = 0;
    }
    else if (_sackDataLength > 0)
    {
        // Transmitter received a SACK packet and is reading it.
        uint8_t* intData = reinterpret_cast<uint8_t*>(data);
        for (uint8_t i = 0; i < _sackDataLength - 2; i++)
        {
            intData[i] = _sackData[i + 2];
        }
        _sackDataLength = 0;
    }
    else
    {
        // Determine length of data in the RX FIFO buffer.
        uint8_t length;
        spiTransfer(READ_OPERATION, R_RX_PL_WID, &length, 1);

        // Read data from the RX FIFO buffer.
        spiTransfer(READ_OPERATION, R_RX_PAYLOAD, data, length);

        // Clear data received flag.
        writeRegister(STATUS, readRegister(STATUS) | _BV(RX_DR));
    }
}

uint8_t NRFLite::send(uint8_t toRadioId, void *data, uint8_t length, SendType sendType)
{
    // Clear any previously asserted TX success or max retries flags.
    uint8_t statusReg = readRegister(STATUS);
    if (statusReg & _BV(TX_DS) || statusReg & _BV(MAX_RT))
    {
        writeRegister(STATUS, statusReg | _BV(TX_DS) | _BV(MAX_RT));
    }
    
    if (sendType == REQUIRE_SACK) 
    {
        // Send the data and wait for a SACK packet (software-based ACK packet).
        return sendSackData(toRadioId, data, length);
    }
    else
    {
        prepForTx(toRadioId, sendType);

        if (sendType == REQUIRE_ACK) { spiTransfer(WRITE_OPERATION, W_TX_PAYLOAD, data, length); }
        else if (sendType == NO_ACK) { spiTransfer(WRITE_OPERATION, W_TX_PAYLOAD_NO_ACK, data, length); }

        // Start transmission.
        // If we have separate pins for CE and CSN, CE will be LOW and we must pulse it to start transmission.
        // If we use the same pin for CE and CSN, CE will already be HIGH and transmission will have started
        // when data was loaded into the TX FIFO buffer.
        if (_cePin != _csnPin)
        {
            digitalWrite(_cePin, HIGH);
            delayMicroseconds(CE_TRANSMISSION_MICROS);
            digitalWrite(_cePin, LOW);
        }

        while (1)
        {
            delayMicroseconds(_transmissionRetryWaitMicros);
            statusReg = readRegister(STATUS);

            if (statusReg & _BV(TX_DS))
            {
                writeRegister(STATUS, statusReg | _BV(TX_DS));   // Clear TX success flag.
                return 1;                                        // Return success.
            }
            else if (statusReg & _BV(MAX_RT))
            {
                spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0); // Clear TX FIFO buffer.
                writeRegister(STATUS, statusReg | _BV(MAX_RT));  // Clear flag which indicates max retries has been reached.
                return 0;                                        // Return failure.
            }
        }
    }
}

void NRFLite::startSend(uint8_t toRadioId, void *data, uint8_t length, SendType sendType)
{
    prepForTx(toRadioId, sendType);

    // Add data to the TX FIFO buffer, with or without an ACK request.
    if (sendType == NO_ACK) { spiTransfer(WRITE_OPERATION, W_TX_PAYLOAD_NO_ACK, data, length); }
    else                    { spiTransfer(WRITE_OPERATION, W_TX_PAYLOAD       , data, length); }
    
    // Start transmission.
    if (_cePin != _csnPin)
    {
        digitalWrite(_cePin, HIGH);
        delayMicroseconds(CE_TRANSMISSION_MICROS);
        digitalWrite(_cePin, LOW);
    }
}

void NRFLite::whatHappened(uint8_t &txOk, uint8_t &txFail, uint8_t &rxReady)
{
    uint8_t statusReg = readRegister(STATUS);
    
    txOk = statusReg & _BV(TX_DS);
    txFail = statusReg & _BV(MAX_RT);
    rxReady = statusReg & _BV(RX_DR);

    // When we need to see interrupt flags, we disable the logic here which clears them.
    // Programs that have an interrupt handler for the radio's IRQ pin will use 'whatHappened'
    // and if we don't disable this logic, it's not possible for us to check these flags.
    if (_resetInterruptFlags)
    {
        writeRegister(STATUS, statusReg | _BV(TX_DS) | _BV(MAX_RT) | _BV(RX_DR));
    }
}

void NRFLite::powerDown()
{
    // If we have separate CE and CSN pins, we can gracefully stop listening or transmitting.
    if (_cePin != _csnPin) { digitalWrite(_cePin, LOW); }
    
    // Turn off the radio.  Only consumes around 900 nA in this state!
    writeRegister(CONFIG, readRegister(CONFIG) & ~_BV(PWR_UP));
}

void NRFLite::printDetails()
{
    printRegister("CONFIG", readRegister(CONFIG));
    printRegister("EN_AA", readRegister(EN_AA));
    printRegister("EN_RXADDR", readRegister(EN_RXADDR));
    printRegister("SETUP_RETR", readRegister(SETUP_RETR));
    printRegister("RF_CH", readRegister(RF_CH));
    printRegister("RF_SETUP", readRegister(RF_SETUP));
    printRegister("STATUS", readRegister(STATUS));
    printRegister("OBSERVE_TX", readRegister(OBSERVE_TX));
    printRegister("FIFO_STATUS", readRegister(FIFO_STATUS));
    printRegister("DYNPD", readRegister(DYNPD));
    printRegister("FEATURE", readRegister(FEATURE));
    
    uint8_t data[5];
    
    String msg = "TX_ADDR ";
    readRegister(TX_ADDR, &data, 5);
    for (uint8_t i = 4; i > 0; i--) { msg += data[i]; msg += ','; }
    msg += data[0];
    
    msg += "\nRX_ADDR_P0 ";
    readRegister(RX_ADDR_P0, &data, 5);
    for (uint8_t i = 4; i > 0; i--) { msg += data[i]; msg += ','; }
    msg += data[0];

    msg += "\nRX_ADDR_P1 ";
    readRegister(RX_ADDR_P1, &data, 5);
    for (uint8_t i = 4; i > 0; i--) { msg += data[i]; msg += ','; }
    msg += data[0];

    msg += "\nRX_ADDR_P2 ";
    readRegister(RX_ADDR_P1, &data, 5);
    for (uint8_t i = 4; i > 0; i--) { msg += data[i]; msg += ','; }
    msg += readRegister(RX_ADDR_P2);

    debugln(msg);
}

/////////////////////
// Private methods //
/////////////////////

uint8_t NRFLite::getPipeOfFirstRxPacket()
{
    // The pipe number is bits 3, 2, and 1.  So B1110 masks them and we shift right by 1 to get the pipe number.
    // 000-101 = Data Pipe Number
    //     110 = Not Used
    //     111 = RX FIFO Empty
    return (readRegister(STATUS) & B1110) >> 1;
}

uint8_t NRFLite::getRxPacketLength()
{
    // Read the length of the first packet sitting in the RX FIFO buffer.
    uint8_t dataLength;
    spiTransfer(READ_OPERATION, R_RX_PL_WID, &dataLength, 1);

    if (dataLength > 32)
    {
        spiTransfer(WRITE_OPERATION, FLUSH_RX, NULL, 0); // Clear invalid data in the RX FIFO buffer.
        writeRegister(STATUS, readRegister(STATUS) | _BV(TX_DS) | _BV(MAX_RT) | _BV(RX_DR));
        return 0;
    }
    else
    {
        return dataLength;
    }
}

uint8_t NRFLite::prepForRx(uint8_t radioId, Bitrates bitrate, uint8_t channel)
{
    delay(OFF_TO_POWERDOWN_MILLIS);

    _resetInterruptFlags = 1;

    // Transmission speed, retry times, and output power setup.
    // For 2 Mbps or 1 Mbps operation, a 500 uS retry time is necessary to support the max ACK packet size.
    // For 250 Kbps operation, a 1500 uS retry time is necessary.
    // '_allowedDataCheckIntervalMicros' is used to limit how often the radio can be checked to determine if data
    // has been received when CE and CSN share the same pin.  If we don't limit how often the radio is checked,
    // the radio may never be given the chance to receive a packet.  More info about this in the 'hasData' method.
    // '_allowedDataCheckIntervalMicros' was determined by maximizing the transfer bitrate between two 16 MHz ATmega328's
    // using 32 byte payloads and sending back 32 byte ACK packets.

    if (bitrate == BITRATE2MBPS)
    {
        writeRegister(RF_SETUP, B00001110);   // 2 Mbps, 0 dBm output power
        writeRegister(SETUP_RETR, B00011111); // 0001 =  500 uS between retries, 1111 = 15 retries
        _maxHasDataIntervalMicros = 600;      
        _transmissionRetryWaitMicros = 250;   
    }                                         
    else if (bitrate == BITRATE1MBPS)         
    {                                         
        writeRegister(RF_SETUP, B00000110);   // 1 Mbps, 0 dBm output power
        writeRegister(SETUP_RETR, B00011111); // 0001 =  500 uS between retries, 1111 = 15 retries
        _maxHasDataIntervalMicros = 1200;     
        _transmissionRetryWaitMicros = 1000;  
    }                                         
    else                                      
    {                                         
        writeRegister(RF_SETUP, B00100110);   // 250 Kbps, 0 dBm output power
        writeRegister(SETUP_RETR, B01011111); // 0101 = 1500 uS between retries, 1111 = 15 retries
        _maxHasDataIntervalMicros = 8000;
        _transmissionRetryWaitMicros = 1500;
    }

    // Valid channel range is 2400 - 2525 MHz, in 1 MHz increments.
    if (channel > 125) { channel = 125; }
    writeRegister(RF_CH, channel);

    // Enable RX pipes and enable dynamically sized packets.
    // Pipe 0 = receives ACK packets (hardware-based ACK packets).
    // Pipe 1 = receives REQUIRE_ACK and NO_ACK packets.
    // Pipe 2 = receives REQUIRE_SACK packets (software-based ACK packets).
    writeRegister(EN_RXADDR, _BV(ERX_P0) | _BV(ERX_P1) | _BV(ERX_P2));
    writeRegister(DYNPD, _BV(DPL_P0) | _BV(DPL_P1) | _BV(DPL_P2));

    // Assign radio addresses.
    // Pipe 1 uses the provided radio id.
    // Pipe 2 uses the provided radio id, plus 100.  If we receive a packet for this address,
    // we know we are dealing with a software-based ACK packet and will perform software-based acknowledgement.
    if (radioId > 99) { radioId = 99; } // Limit valid radio id range 0 - 99.
    _radioId = radioId;
    uint8_t address[5] = { _radioId, 40, 30, 20, 10 };
    writeRegister(RX_ADDR_P1, &address, 5);
    writeRegister(RX_ADDR_P2, _radioId + 100);

    // Enable dynamically sized payloads, ACK payloads, and TX support with or without an ACK request.
    writeRegister(FEATURE, _BV(EN_DPL) | _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK));

    // Ensure RX FIFO and TX FIFO buffers are empty.  Each buffer can hold 3 packets.
    spiTransfer(WRITE_OPERATION, FLUSH_RX, NULL, 0);
    spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0);

    // Clear any interrupts.
    uint8_t statusReg = readRegister(STATUS);
    writeRegister(STATUS, statusReg | _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    // Power on the radio and start listening, delaying to allow startup to complete.
    uint8_t newConfigReg = _BV(EN_CRC) | _BV(PWR_UP) | _BV(PRIM_RX);
    writeRegister(CONFIG, newConfigReg);
    digitalWrite(_cePin, HIGH);
    delayMicroseconds(POWERDOWN_TO_RXTX_MODE_MICROS);

    // Return success if the update we made to the CONFIG register was successful.
    return readRegister(CONFIG) == newConfigReg;
}

void NRFLite::prepForTx(uint8_t toRadioId, SendType sendType)
{
    if (sendType == REQUIRE_SACK)
    {
        toRadioId += 100; // Send to the receiving radio's pipe 2 address to indicate it must send a software-based ACK.
    }

    uint8_t address[5] = { toRadioId, 40, 30, 20, 10 };
    writeRegister(TX_ADDR, &address, 5);    // Transmit to the radio with this address.
    writeRegister(RX_ADDR_P0, &address, 5); // Receive hardware-based ACK on pipe 0.

    // Ensure radio is powered on and ready for TX operation.
    uint8_t configReg = readRegister(CONFIG);
    uint8_t newConfigReg = configReg & ~_BV(PRIM_RX) | _BV(PWR_UP);
    if (configReg != newConfigReg)
    {
        // In case the radio was in RX mode (powered on and listening), we'll put the radio into
        // Standby-I mode by setting CE LOW.  The radio cannot transition directly from RX to TX,
        // it must go through Standby-I first.
        if ((configReg & _BV(PRIM_RX)) && (configReg & _BV(PWR_UP)))
        {
            if (digitalRead(_cePin) == HIGH) { digitalWrite(_cePin, LOW); }
        }
        
        writeRegister(CONFIG, newConfigReg);
        delayMicroseconds(STANDBY_TO_RXTX_MODE_MICROS);

        // If radio was powered off, wait for it to turn on.
        if ((configReg & _BV(PWR_UP)) == 0)
        {
            delayMicroseconds(POWERDOWN_TO_RXTX_MODE_MICROS);
        }
    }
    
    // If RX FIFO buffer is full and we require an ACK, clear it so we can receive the ACK response.
    uint8_t fifoReg = readRegister(FIFO_STATUS);
    if (fifoReg & _BV(RX_FULL) && (sendType == REQUIRE_ACK || sendType == REQUIRE_SACK))
    {
        spiTransfer(WRITE_OPERATION, FLUSH_RX, NULL, 0);
    }

    // If TX FIFO buffer is full, we'll attempt to send all the packets it contains.
    if (fifoReg & _BV(FIFO_FULL))
    {
        // Disable interrupt flag reset logic in 'whatHappened' so we can use the flags here.
        _resetInterruptFlags = 0;
        uint8_t statusReg;
        
        // While the TX FIFO buffer is not empty...
        while (!(fifoReg & _BV(TX_EMPTY)))
        {
            // Try sending a packet.
            digitalWrite(_cePin, HIGH);
            delayMicroseconds(CE_TRANSMISSION_MICROS);
            digitalWrite(_cePin, LOW);
            
            delayMicroseconds(_transmissionRetryWaitMicros);
            statusReg = readRegister(STATUS);

            if (statusReg & _BV(TX_DS))
            {
                writeRegister(STATUS, statusReg | _BV(TX_DS));   // Clear TX success flag.
            }
            else if (statusReg & _BV(MAX_RT))
            {
                spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0); // Clear TX FIFO buffer.
                writeRegister(STATUS, statusReg | _BV(MAX_RT));  // Clear flag which indicates max retries has been reached.
            }

            fifoReg = readRegister(FIFO_STATUS);
        }
        
        _resetInterruptFlags = 1;
    }
}

uint8_t NRFLite::getSackDataLengthAndSendAck()
{
    _receivedDataLength = getRxPacketLength();

    if (_receivedDataLength > 0)
    {
        // Read data from RX FIFO buffer.
        spiTransfer(READ_OPERATION, R_RX_PAYLOAD, &_requireSackData, _receivedDataLength);

        // Clear data received flag.
        writeRegister(STATUS, readRegister(STATUS) | _BV(RX_DR)); 

        delay(SACK_TX_TO_RX_MILLIS); // Wait for transmitter to become a receiver.

        if (_sackDataLength > 0)
        {
            // Send user-provided data (added via 'addAckData').
            uint8_t toRadioId = _requireSackData[0];
            send(toRadioId, &_sackData, _sackDataLength, NRFLite::NO_ACK);
        }
        else
        {
            // Send non-data packet (only contains our radio id and the packet id being acknowledged).
            uint8_t toRadioId = _requireSackData[0];
            uint8_t dataId = _requireSackData[1];
            uint16_t nonDataPacketId = _radioId << 8 | dataId;
            send(toRadioId, &nonDataPacketId, 2, NRFLite::NO_ACK);
        }
    }

    // Ignore the packet if we already received it.
    uint16_t receivedPacketId = _requireSackData[0] << 8 | _requireSackData[1];
    if (receivedPacketId == _lastPacketId)
    {
        _receivedDataLength = 0;
        return 0;
    }
    else
    {
        _lastPacketId = receivedPacketId;
        return _receivedDataLength - 2; // Exclude the 2 byte packet id.
    }
}

uint8_t NRFLite::sendSackData(uint8_t toRadioId, void *data, uint8_t length)
{
    if (length > 30) length = 30; // Limit user's data to 30 bytes.

    // Create a byte array for the REQUIRE_SACK (software-based acknowledgement) packet to send.
    // Byte 0 = our radio id, byte 1 = data id, bytes 2-32 are the user's data.
    // The receiver uses byte 0 to know where it should transmit back the acknowledgement and byte 1
    // to know if it has already received the data and should ignore it.
    uint8_t sackData[length + 2];
    sackData[0] = _radioId;
    sackData[1] = _lastRequireSackDataId++;
    uint8_t* intData = reinterpret_cast<uint8_t*>(data);
    for (uint8_t i = 0; i < length; i++)
    {
        sackData[i + 2] = intData[i];
    }

    uint8_t retryCount = 0, successFlag = 0;
    _sackDataLength = 0; // Stores the size of any SACK packet returned by the receiver.

    while (retryCount++ < 16)
    {
        // Clear any previously asserted TX flags.
        uint8_t statusReg = readRegister(STATUS);
        if (statusReg & _BV(TX_DS) || statusReg & _BV(MAX_RT))
        {
            writeRegister(STATUS, statusReg | _BV(TX_DS) | _BV(MAX_RT));
        }

        // Change to TX operation and send data.
        prepForTx(toRadioId, REQUIRE_SACK);
        spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0);
        spiTransfer(WRITE_OPERATION, W_TX_PAYLOAD_NO_ACK, sackData, length + 2);

        if (_cePin != _csnPin)
        {
            digitalWrite(_cePin, HIGH);
            delayMicroseconds(CE_TRANSMISSION_MICROS);
            digitalWrite(_cePin, LOW);
        }
        
        delay(SACK_TX_COMPLETE_MILLIS);

        // Change to RX operation.
        writeRegister(CONFIG, readRegister(CONFIG) | _BV(PRIM_RX));

        if (_cePin != _csnPin)
        {
            if (digitalRead(_cePin) == LOW) digitalWrite(_cePin, HIGH);
        }

        delay(SACK_RX_WAIT_TIME_MILLIS);

        if (getPipeOfFirstRxPacket() == 1) // Receiver transmits acknowledgement back to our pipe 1 address.
        {
            successFlag = 1;
            _sackDataLength = getRxPacketLength();

            if (_sackDataLength == 2)
            {
                // Received non-data packet.
                spiTransfer(WRITE_OPERATION, FLUSH_RX, NULL, 0);
                writeRegister(STATUS, readRegister(STATUS) | _BV(RX_DR));
                _sackDataLength = 0; // Clear the indication of SACK data.
            }
            else
            {
                // Received data-bearing SACK packet.
                // The transmitter will be able to check for this data using 'hasAckData'.
                spiTransfer(READ_OPERATION, R_RX_PAYLOAD, &_sackData, _sackDataLength);
            }
            
            break;
        }
    }

    if (!successFlag)
    {
        spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0); // Clear TX FIFO buffer.
    }

    return successFlag;
}

uint8_t NRFLite::readRegister(uint8_t regName)
{
    uint8_t data;
    readRegister(regName, &data, 1);
    return data;
}

void NRFLite::readRegister(uint8_t regName, void *data, uint8_t length)
{
    spiTransfer(READ_OPERATION, (R_REGISTER | (REGISTER_MASK & regName)), data, length);
}

void NRFLite::writeRegister(uint8_t regName, uint8_t data)
{
    writeRegister(regName, &data, 1);
}

void NRFLite::writeRegister(uint8_t regName, void *data, uint8_t length)
{
    spiTransfer(WRITE_OPERATION, (W_REGISTER | (REGISTER_MASK & regName)), data, length);
}

void NRFLite::spiTransfer(SpiTransferType transferType, uint8_t regName, void *data, uint8_t length)
{
    uint8_t* intData = reinterpret_cast<uint8_t*>(data);

    if (_useTwoPinSpiTransfer)
    {
        digitalWrite(_csnPin, LOW);              // Signal radio it should begin listening to the SPI bus.
        delayMicroseconds(CSN_DISCHARGE_MICROS); // Allow capacitor on CSN pin to discharge.
        noInterrupts();                          // Timing is critical so interrupts are disabled during the bit-bang transfer.
        twoPinTransfer(regName);
        for (uint8_t i = 0; i < length; ++i) {
            uint8_t newData = twoPinTransfer(intData[i]);
            if (transferType == READ_OPERATION) { intData[i] = newData; }
        }
        interrupts();
        digitalWrite(_csnPin, HIGH);             // Stop radio from listening to the SPI bus.
        delayMicroseconds(CSN_DISCHARGE_MICROS); // Allow capacitor on CSN pin to recharge.
    }
    else
    {
        digitalWrite(_csnPin, LOW); // Signal radio it should begin listening to the SPI bus.

        #if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
            // ATtiny transfer with USI.
            usiTransfer(regName);
            for (uint8_t i = 0; i < length; ++i) {
                uint8_t newData = usiTransfer(intData[i]);
                if (transferType == READ_OPERATION) { intData[i] = newData; }
            }
        #else
            // Transfer with the Arduino SPI library.
            SPI.transfer(regName);
            for (uint8_t i = 0; i < length; ++i) {
                uint8_t newData = SPI.transfer(intData[i]);
                if (transferType == READ_OPERATION) { intData[i] = newData; }
            }
        #endif

        digitalWrite(_csnPin, HIGH); // Stop radio from listening to the SPI bus.
    }
}

uint8_t NRFLite::usiTransfer(uint8_t data)
{
    #if defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    
        USIDR = data;
        USISR = _BV(USIOIF);
    
        while ((USISR & _BV(USIOIF)) == 0)
        {
            USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK) | _BV(USITC);
        }
    
        return USIDR;
    
    #endif
}

uint8_t NRFLite::twoPinTransfer(uint8_t data)
{
    uint8_t byteFromRadio;
    uint8_t bits = 8;
    
    do
    {
        byteFromRadio <<= 1; // Shift the byte we are building to the left.
        
        if (*_momi_PIN & _momi_MASK) { byteFromRadio++; } // Read bit from radio on MOMI pin.  If HIGH, set bit position 0 of our byte to 1.
        *_momi_DDR |= _momi_MASK;                         // Change MOMI to be an OUTPUT pin.
        
        if (data & 0x80) { *_momi_PORT |=  _momi_MASK; }  // Set MOMI HIGH if bit position 7 of the byte we are sending is 1.

        *_sck_PORT |= _sck_MASK;  // Set SCK HIGH to transfer the bit to the radio.  CSN will remain LOW while the capacitor begins charging.
        *_sck_PORT &= ~_sck_MASK; // Set SCK LOW.  CSN will have remained LOW due to the capacitor.
        
        *_momi_PORT &= ~_momi_MASK; // Set MOMI LOW.
        *_momi_DDR &= ~_momi_MASK;  // Change MOMI back to an INPUT.  Since we previously ensured it was LOW, its PULLUP resistor will never 
                                    // be enabled which would prevent MOMI from fully reaching a LOW state.

        data <<= 1; // Shift the byte we are sending to the left.
    }
    while (--bits);
    
    return byteFromRadio;
}

void NRFLite::printRegister(char name[], uint8_t reg)
{
    String msg = name;
    msg += " ";

    for (int8_t i = 7; i >= 0; i--)
    {
        msg += bitRead(reg, i);
    }

    debugln(msg);
}
