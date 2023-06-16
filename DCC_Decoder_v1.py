# DCC accessory decoder in micropython for Raspberry Pi Pico
# Chris Hall, MERG 3011
# Distributed under Creative Commons licence "Creative Commons Zero v1.0 Universal"
# Not to be used in for-profit applications
# note set up for LENZ address space which differs for other manufacturers
# 15 Jun 23 15:52
from machine import Pin
import array
from rp2 import PIO, StateMachine, asm_pio
import time

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
output_pin = array.array('i',(i for i in range(0,4)))
output_pin = [17,18,19,20] # set for user preference
function_pin = ["0","1","2","3"]


#CVs



# Decoder Address CV1 , 6 MSB of accessory decoder address

CV1 = 10 # CV513  = for Lenz this is switch 4 /10

# Auxiliary Activation CV2  Auxiliary activation of outputs

CV2 = 255 # CV 514 all enabled - not used in v1

CV3 = 0 # Time On F1  - not used in v1
CV4 = 0 # Time On F2  - not used in v1
CV5 = 0 # Time On F3  - not used in v1
CV6 = 0 # Time On F4  - not used in v1

#Manufacturer Version
CV7 = 1 # Manufacturer defined version info
# Manufacturer ID   - Values assigned by NMRA - Query - does MERG have a mfg no?
CV8 = 0

# Decoder Address - 3 Least SB of accessory decoder address NOTE not used by LENZ

CV9 = 0 # CV 521

#Bi-Directional Communication Configuration
CV28 = 0 # no return path
# Accessory Decoder Configuration

CV29 = 0 #similar to CV#29; for acc.decoders 



# initialise pin states
i=0
while i <4:
    function_pin[i] = Pin(output_pin[i], Pin.OUT, value = 0)
    i=i+1

pinIN = Pin(input_pin, Pin.IN, Pin.PULL_UP)
sm0 = rp2.StateMachine(0, period, in_base=pinIN, jmp_pin=pinIN)
sm0.active(1)

# Clock is 125MHz. 3 cycles per iteration, so unit is 24.0ns
def scale(v):
    return (1 + (v ^ 0xffffffff)) * 24e-6  # Scale to ns

data = array.array('b', (0 for _ in range(1000))) # diagnostic only - last 1000 bits


address = array.array ('b', (0 for _ in range (8))) # MSB leads to extended address
command1 = array.array ('b', (0 for _ in range (8))) # details of command byte
command2 = array.array ('b', (0 for _ in range (8))) # future
command3 = array.array ('b', (0 for _ in range (8))) # future
checksum = array.array ('b', (0 for _ in range (8))) # future
checksum_check = array.array ('b', (0 for _ in range (8)))

# error counters for diagnostics
checksumfailcount = 0
commandfailcount = 0
addressfailcount = 0
risetimefailcount = 0

cyclecount = 0
flagcount = 0
index = 0

def fetchbit(index, failcount):
    if index <1000:
        index = index + 1
    else:
        index = 1
    
    bitfound = 0
    while bitfound == 0:
        risefall =  scale(sm0.get())
    
        if risefall > 0.1 : #allows for stretched zeroes
            data [index] = 0
            bitfound = 1
            return (0)
        elif risefall > 0.055:
            data [index] = 1
            bitfound = 1
            return (1)
        else:
            #printprint ("rise time :", risefall) # too short for a 1
            failcount = failcount +1
        
        

def byteload (data_byte, index):
    #load a byte with data from the input
    i = 0
    while i < 8:
        data_byte [i] = fetchbit (index, risetimefailcount)
        i=i+1       
    return (data_byte,index)

# this returns a byte value representing xor of '0' and '1' byte values
def byte_xor(ba1, ba2):
    if ba1 == 0:
        if ba2 == 0:
            return  0
        else:
            return  1
    else:
        if ba2 == 1:
            return  0
        else:
            return  1
        

while True: # print data until control-C
        
    nextbit = fetchbit (cyclecount, risetimefailcount)
       
    # detect a zero - if flag detected then this is packet start bit
    if nextbit == 0: # at least 100ms allows for stretched zeros as well
        if flagcount >= 10 and flagcount<=14: # at least 10 bits but less than 15
            #print ("FLAG ", flagcount, " bits", "cycles ", cyclecount)
            flagcount = 0
            
            #pick up address
            byteload (address, cyclecount)
            
            converted_address = 0 #convert to integer
   
            #print ("address:",address)
            
            # check this is for an accessory decoder
            # NMRA spec
            #Basic Accessory Decoder Packet Format
# The format for packets intended for Accessory Digital Decoders is:
# {preamble} 0 10AAAAAA 0 1AAACDDD 0 EEEEEEEE 1
            
            if address [0] == 1 and address[1] == 0:
            # convert address to integer
                i = 2
                while i <8:
                    converted_address = converted_address  +((address[i] & 1) << (7-i)) # only convert LS 6 bits
                    i = i +1
                    #pick up command byte which includes function no - for Lenz this is 2 LSB of address
                    # note error in spec - Lenz sends 1010 in address byte for address 40
            
                #print ("MSBs",converted_address)
                if converted_address == CV1:  # first bits of address found
                    # pico controls 4 outputs addressed by bits 5,6 in command byte
                
                    end_address_bit = fetchbit (cyclecount,risetimefailcount)
            
                    if end_address_bit != 0 :
                        #print ("Error bit: after address non-zero : pulse length", period) # could be end of packet?
                        addressfailcount = addressfailcount +1
                    byteload (command1,cyclecount)
       
           
                        #print ("Address : ", converted_address, "   ", address, end= ": ")
                        #print ("Command byte 1:",command1)
            
                    # look for checksum byte
     
                    end_command_bit = fetchbit (cyclecount,risetimefailcount)
                    if end_command_bit !=0 :
                        #print ("Error bit: after command non-zero : pulse length", period) # could be end of packet?
                        commandfailcount = commandfailcount+1
                        end_command_bit = 1
                    byteload (checksum, cyclecount)
            
                    i = 0
                    flag = 0
                    while i<8:
                        checksum_check [i] = byte_xor (address [i] , command1 [i])
                        if checksum_check [i] != checksum[i]:
                            flag = 1
                    
                        i=i+1
                
                    #else:
                        #print ("Checksum OK :" , checksum)
                    # look for terminal bit
                    terminal_bit = fetchbit (cyclecount,risetimefailcount)
                    if terminal_bit != 1:
                        #print ("Error, bit: after checksum  is zero : pulse length", period)
                        checksumfailcount = checksumfailcount+1
                    if flag ==1 :
                        print ("Checksum FAIL :" , address, end_address_bit, command1, end_command_bit, checksum, terminal_bit)
                    else:
                        #print ("Checksum OK :" , address, end_address_bit, command1,end_command_bit, checksum,terminal_bit)
                        # checksum OK so can execute function - get address of function from command byte bits 5,6
                        function_number = 2*command1[5]+command1[6]
                        #print ("switch number:", 4*(converted_address-1)+ function_number+1)
                        if command1[7] == 1:
                            function_pin [function_number].on()
                        else:
                            function_pin [function_number].off()
                    flagcount = 0 # start looking for flags again
# add subsequent command bytes later    
        else:
            flagcount = 0 #start looking again for flags (preamble bits)
                #print ("Zero no flag") # diagnostic to see how many short pulses in stream
        
    else:
        if nextbit == 1: # 55 ms - anything shorter ignored
            flagcount = flagcount + 1
    
   
# this version should loop continuously

    

