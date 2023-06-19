# DCC accessory decoder in micropython for Raspberry Pi Pico
# Chris Hall, MERG 3011
# Distributed under Creative Commons licence "Creative Commons Zero v1.0 Universal"
# Not to be used in for-profit applications
# note set up for LENZ address space which differs for other manufacturers
# V2 19 Jun 23
"""Changes from V1

removed floating point arithmetic
uses for loops rather than incrementing while loops
tidy up comments
removed 'data' array which was a rolling cache for diagnostic purposes
removed other redundant code
"""

from machine import Pin, PWM
import array
from rp2 import PIO, StateMachine, asm_pio
import time
import math

# thanks to pythoncoder for (much of) peak-to-peak measuring code

@rp2.asm_pio(set_init=rp2.PIO.IN_LOW, autopush=True, push_thresh=32)
def period():
    wait(0, pin, 0)  # Wait for pin to go low - init condition
    wrap_target()
    set(x, 0)
    # this version assumes half-rectified sq wave and only counts highs
    # assumes pin low at entry
    label ('entry')
    jmp (pin,'pin_still_high') # detect pin high and start counting
    # may not need this and wait cmd
    jmp ('entry')
    label('pin_still_high')
    jmp(x_dec, 'next') [1]  # unconditional (but why decrement?)
    label('next')
    jmp(pin, 'pin_still_high') # while pin is high loop back
    in_(x, 32)  # Auto push: SM stalls if FIFO full
    wrap()

# CONSTANTS FOR ACCESSORIES ETC

input_pin = 16 # set for user preference

output_pin = [17,18,19,20] # set for user preference
function_pin = ["0","1","2","3"]

#CVs (not all of which used and none externally addressable

# Decoder Address CV1 , 6 MSB of accessory decoder address

CV1 = 1 # CV513  = for Lenz CV1 = 10 is for switches 37,38,39 and 40.  This CV can be in range 1-63 for Lenz

# Auxiliary Activation CV2  Auxiliary activation of outputs

CV2 = 255 # CV 514 all enabled - not used in v1

CV3 = 0 # Time On F1  - not used in v1 - constantly on
CV4 = 0 # Time On F2  - not used in v1 - constantly on
CV5 = 0 # Time On F3  - not used in v1 - constantly on
CV6 = 0 # Time On F4  - not used in v1 - constantly on

#Manufacturer Version
CV7 = 1 # Manufacturer defined version info
# Manufacturer ID   - Values assigned by NMRA - Query - can we use MERG no?
CV8 = 0

# Decoder Address - 3 Least SB of accessory decoder address NOTE not used by LENZ

CV9 = 0 # CV 521

#Bi-Directional Communication Configuration
CV28 = 0 # no return path

# Accessory Decoder Configuration

CV29 = 0 #similar to CV#29 for normal decoders 

# initialise pin states

for i in range (4):
    function_pin[i] = Pin(output_pin[i], Pin.OUT, value = 0)

# set up input pin for PIO
pinIN = Pin(input_pin, Pin.IN, Pin.PULL_UP)
sm0 = rp2.StateMachine(0, period, in_base=pinIN, jmp_pin=pinIN)
sm0.active(1)

# arrays for packet bytes rather than lists so no dynamic memory allocation???
# These are themselves integer arrays, each of which represents a bit
# MSB is in {array} [0] as it is the first bit received
address = array.array ('i', (0 for _ in range (8))) # MS 2 bits '10' signifies decoder address 
command1 = array.array ('i', (0 for _ in range (8))) # command byte has LS bits of address  - see notes on Lenz
command2 = array.array ('i', (0 for _ in range (8))) # future
command3 = array.array ('i', (0 for _ in range (8))) # future
checksum = array.array ('i', (0 for _ in range (8))) # error byte

# error counters for diagnostics- can br printed after control-C and indicate how cleanly bitstream is being read
checksumfailcount = 0
commandfailcount = 0
addressfailcount = 0
risetimefailcount = 0

flagcount = 0  # counts consecutive 1s - intent is to find at least 10 consecutive for preamble

def fetchbit(failcount):
# get rise time from the PIO buffer and from rise time decide whether one or zero  
    bitfound = False
    while bitfound == False and failcount <1000: # allow for unconnected card ??? maybe return 'none' and let next level decide
        risefall =  sm0.get()^ 0xffffffff
        if risefall > 4300 : # 0.1ms of greater allows for stretched zeroes 
            bitfound = True
            return (0)
        elif risefall > 2300: # 0.55ms
            bitfound = True
            return (1)
        else:
            #print ("rise time :", risefall) # too short for a 1
            failcount = failcount +1
            # if rise time too short this version just loops back and keeps looking for a longer bit until failcount hits 1000
               
def byteload (data_byte):
#load a byte with data from the input   
    for i in range (8):
        data_byte [i] = fetchbit (risetimefailcount)      
    return (data_byte)

def byte_xor(x, y):
# return a byte value representing xor of '0' and '1' byte values    
    if x == y:
        return  0
    else:
        return  1
    
# main loop
while True: # loop reading data until control-C       
    nextbit = fetchbit (risetimefailcount)    
    # detect a zero - if flag precedes this zero then this is packet start bit
    if nextbit == 0: 
        if flagcount >= 10 and flagcount<=14: # flag has at least 10 bits but less than 15
            #print ("FLAG ", flagcount, " bits")
            flagcount = 0
            # yes, this is packet start bit
            # pick up address as first byte of packet     
            byteload (address)
     
            #print ("address:",address)
            
            # Check this is for an accessory decoder
            # NMRA spec
            # Basic Accessory Decoder Packet Format
            # The format for packets intended for Accessory Digital Decoders is:
            # {preamble} 0 10AAAAAA 0 1AAACDDD 0 EEEEEEEE 1
            # Extended packet has up to 5 bytes
            # This version discards packets with incorrect checksum
            # but tolerates incorrect 'interbyte' bits - after address, command and checksum bytes
            
            converted_address = 0 #convert LS 6 bits of address byte to integer
            if address [0] == 1 and address[1] == 0: # this is an accessory decoder
            # convert address to integer
                for i in range(2,8):
                    converted_address = converted_address + ((address[i] & 1) << (7-i)) # only convert LS 6 bits
                
                # pick up command byte which includes function no - for Lenz this is 2 LSB of switch number eg switch 40 has function no 3
                # note difference from NMRA spec - Lenz sends 001010 in address byte for switch addresses 37-40
   
                #print ("MSBs",converted_address)
                if converted_address == CV1:  # first bits of address found
                    # pico controls 4 GPIO outputs addressed by bits 5,6 in command byte
                    end_address_bit = fetchbit (risetimefailcount)
                    if end_address_bit != 0 :
                        #print ("Error bit after address non-zero ") # could be end of packet?
                        addressfailcount = addressfailcount +1
                    # load second byte of packet which is first command byte
                    byteload (command1)
                    # Command byte has format 1AAACDDD
                    # This code does not check MS 5 bits
                    # LS 3 bits DDD - LS bit is on or off, bits 6,7 are 'function' address
                    #print ("Address : ", converted_address, "   ", address, end= ": ")
                    #print ("Command byte 1:",command1)
            
                    # retrieve checksum byte   
                    end_command_bit = fetchbit (risetimefailcount)
                    if end_command_bit !=0 :
                        #print ("Error bit: after command non-zero : pulse length", period) # could be end of packet?
                        commandfailcount = commandfailcount+1
                        end_command_bit = 1
                    byteload (checksum)
                    checksum_error_flag = False
                    for i in range(8):
                        if checksum[i] != byte_xor (address[i] , command1[i]):
                            checksum_error_flag = True
                        #else:
                            #print ("Checksum OK :" , checksum)
                    # look for terminal bit should be 1, signals end of packet
                    terminal_bit = fetchbit (risetimefailcount)
                    if terminal_bit != 1:
                        #print ("Error, bit: after checksum  is zero)
                        checksumfailcount = checksumfailcount+1
                   
                    if checksum_error_flag:
                        print ("Checksum FAIL :" , address, end_address_bit, command1, end_command_bit, checksum, terminal_bit)
                    else:
                        #print ("Checksum OK :" , address, end_address_bit, command1,end_command_bit, checksum,terminal_bit)
                        # checksum OK so can execute function - get address of function from command byte bits 5,6
                        function_number = 2 * command1[5] + command1[6]
                        #print ("switch number:", 4 * (converted_address-1) + function_number + 1)
                        
                        if command1[7] == 0:
                            function_pin [function_number].off()
                        else:
                            function_pin [function_number].on()
                    flagcount = 0 # start looking for flags again
# add subsequent command bytes in later version    
        else:
            flagcount = 0 #start looking again for flags (preamble bits)
                #print ("Zero no flag") # diagnostic to see how many short pulses in stream
        
    else:
        if nextbit == 1: # possible preamble bit
            flagcount = flagcount + 1
    
   
# this version should loop continuously until Control-C

 





   
