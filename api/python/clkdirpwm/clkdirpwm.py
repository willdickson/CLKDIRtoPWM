#!/usr/bin/env python
#
# clkdirpwm.py 
#
# Control interface for at90usb based clock and direction to pwm board. 
# Provides python module and a command line utility.
#
# Note, need to set permissions correctly to get device to respond to nonroot
# users. This required adding and rules file to udev/rules.d and adding a 
# group. 
#
#    who when        what
#    --- ----        ----
#    pjp 07/15/08    version 1.0
# --------------------------------------------------------------------------- 
import pylibusb as usb
import ctypes
import sys
import time
import optparse

DEBUG = False

# USB device parameters
USB_VENDOR_ID = 0x1781 
USB_PRODUCT_ID = 0x0BB3
USB_BULKOUT_EP_ADDRESS = 0x01
USB_BULKIN_EP_ADDRESS = 0x82
USB_BUFFER_SIZE = 64

# USB Command IDs
USB_CMD_SERVO_READ = 1
USB_CMD_SERVO_WRITE = 2
USB_CMD_SERVO_ACTIVATE = 3
USB_CMD_PWM_TOGGLE = 4
USB_CMD_PWM_SET_TO_DEFAULT = 5
USB_CMD_AVR_RESET = 200
USB_CMD_AVR_DFU_MODE = 201
USB_CMD_TEST = 251

RUNNING = 1
STOPPED = 0
WAIT_SLEEP_T = 0.1

# Command line defaults
CMDLINE_DEFAULT_VERBOSE = False
CMDLINE_DEFAULT_WAIT = False

def debug(val):
    if DEBUG==True:
        print >> sys.stderr, val

def debug_print(msg, comma=False):
    if DEBUG==True:
        if comma==True:
            print msg, 
        else:
            print msg
        sys.stdout.flush()

class clkdirpwm_device:
    def __init__(self):
        usb.init()
        
        # Get usb busses
        if not usb.get_busses():
            usb.find_busses()            
            usb.find_devices()
        busses = usb.get_busses()

        # Find device by IDs
        found = False
        for bus in busses:
            for dev in bus.devices:
                #print 'idVendor: 0x%04x idProduct: 0x%04x'%(dev.descriptor.idVendor,
                #                                            dev.descriptor.idProduct)
                if (dev.descriptor.idVendor == USB_VENDOR_ID and
                    dev.descriptor.idProduct == USB_PRODUCT_ID):
                    found = True
                    break
            if found:
                break
        if not found:
            raise RuntimeError("Cannot find device.")

        self.libusb_handle = usb.open(dev)
        
        interface_nr = 0
        if hasattr(usb,'get_driver_np'):
            # non-portable libusb function available
            name = usb.get_driver_np(self.libusb_handle,interface_nr)
            if name != '':
                debug("attached to kernel driver '%s', detaching."%name )
                usb.detach_kernel_driver_np(self.libusb_handle,interface_nr)


        if dev.descriptor.bNumConfigurations > 1:
            debug("WARNING: more than one configuration, choosing first")
        
        usb.set_configuration(self.libusb_handle, dev.config[0].bConfigurationValue)
        usb.claim_interface(self.libusb_handle, interface_nr)
        
        self.output_buffer = ctypes.create_string_buffer(USB_BUFFER_SIZE)
        self.input_buffer = ctypes.create_string_buffer(USB_BUFFER_SIZE)
        for i in range(USB_BUFFER_SIZE):
            self.output_buffer[i] = chr(0x00)
            self.input_buffer[i] = chr(0x00)

    def read(self):
        self.output_buffer[0] = chr(USB_CMD_SERVO_READ%0x100)
        data = self._send_and_receive()
        cmd_id = ord(data[0])
        _check_cmd_id(USB_CMD_SERVO_READ, cmd_id)
        return

    def test(self):
        self.output_buffer[0] = chr(USB_CMD_TEST%0x100)
        data = self._send_and_receive()
        cmd_id = ord(data[0])
        _check_cmd_id(USB_CMD_TEST, cmd_id)
        return

    def _send_and_receive(self,in_timeout=1000,out_timeout=9999):
        # Send bulkout and and receive bulkin as a response
        # Note, probably want to and a max count so this will 
        # timeout if all of the reads fail.  
        done = False
        while not done:
            val = self._send_output(timeout=out_timeout)
            data = self._read_input(timeout=in_timeout)
            if data == None:
                debug_print('usb SR: fail', comma=False) 
                sys.stdout.flush()
                continue
            else:
                done = True
                debug_print('usb SR cmd_id: %d'%(ord(data[0]),), comma=False) 
        return data
    
    def _send_output(self,timeout=9999):
        buf = self.output_buffer # shorthand
        #print 'write', [ord(b) for b in buf]
        val = usb.bulk_write(self.libusb_handle, USB_BULKOUT_EP_ADDRESS, buf, timeout)
        return val

    def _read_input(self, timeout=1000):
        buf = self.input_buffer
        try:
            val = usb.bulk_read(self.libusb_handle, USB_BULKIN_EP_ADDRESS, buf, timeout)
            #print 'read', [ord(b) for b in buf]
            data = [x for x in buf]
        except usb.USBNoDataAvailableError:
            data = None
        return data
                
    def close(self):
        ret = usb.close(self.libusb_handle)


    def wait(self):
        while self.get_status()==1:
            time.sleep(WAIT_SLEEP_T)
            pass

    def enter_dfu_mode(self):
        self.output_buffer[0] = chr(USB_CMD_AVR_DFU_MODE%0x100)
        data = self._send_and_receive()
        cmd_id = ord(data[0])
        _check_cmd_id(USB_CMD_AVR_DFU_MODE, cmd_id)
        return

    def reset(self):
        self.output_buffer[0] = chr(USB_CMD_AVR_RESET%0x100)
        data = self._send_and_receive()
        cmd_id = ord(data[0])
        _check_cmd_id(USB_CMD_AVR_RESET, cmd_id)
        return

    def set_pwm_to_default(self):
        self.output_buffer[0] = chr(USB_CMD_PWM_SET_TO_DEFAULT%0x100)
        data = self._send_and_receive()
        cmd_id = ord(data[0])
        _check_cmd_id(USB_CMD_PWM_SET_TO_DEFAULT, cmd_id)
        return

def _check_cmd_id(expected_id,received_id):
    if not expected_id == received_id:
        msg = "received incorrect command ID %d expected %d"%(received_id,expected_id)
        raise IOError, msg




# Commandline interface ---------------------------------------------------------

CLKDIRPWM_USAGE_STR = """
%prog [options] test
%prog [options] reset
%prog [options] dfu-mode
 
%prog provides a command line interface to the usb to clock and direction to pwm 
board based on the at90usb demo-kit. Allows the user to view/change the 
current device settings, start/stop the device output and place the  device 
in dfu programming mode. 


Command Summary:
  test - sends test command to the device.      
  dfu-mode - puts at90usb device into dfu programming mode. 


Examples: 

  %prog read 
  Reads the status of the servos from the device.

  %prog test 
  Sends test command to the device.

  %prog reset
  Performs a software reset of the at90usb device.

  %prog dfu-mode
  Places at90usb device into dfu programming mode.

  %prog pwm-to-default
  Sets all pwm signals to thier default values.

"""

def clkdirpwm_main():
    """
    Main routine for clkdirpwm commandline function. 
    """
    parser = optparse.OptionParser(usage=CLKDIRPWM_USAGE_STR)

    parser.add_option('-v', '--verbose',
                      action='store_true',
                      dest='verbose',
                      help='verbose mode - print addition information',
                      default=CMDLINE_DEFAULT_VERBOSE)

    parser.add_option('-w', '--wait',
                      action='store_true',
                      dest='wait',
                      help='return only after sinewave outscan complete',
                      default=CMDLINE_DEFAULT_WAIT)
    
    options, args = parser.parse_args()
    try:
        command = args[0].lower()
    except:
        print 'E: no command argument'
        sys.exit(1)

    if command=='read':
        read(options)

    elif command=='test':
        test(options)
    
    elif command=='reset':
        reset(options)

    elif command=='dfu-mode':
        dfu_mode(options)

    elif command=='pwm-to-default':
        set_pwm_to_default(options)
        
    else:
        print 'E: uknown command %s'%(command,)
        sys.exit(1)


def dfu_mode(options):
    v = options.verbose  
    # Open device
    vprint('opening device ... ',v,comma=True)
    dev = clkdirpwm_device()
    vprint('done',v)

    vprint('entering dfu mode ... ',v, comma=True)
    dev.enter_dfu_mode()
    vprint('done',v)
           
    # Close device
    vprint('closing device ... ', v, comma=True)
    dev.close()
    vprint('done',v)
    return

def set_pwm_to_default(options):
    try:
        v = options.verbose  
    except:
        v = False
    # Open device
    vprint('opening device ... ',v,comma=True)
    dev = clkdirpwm_device()
    vprint('done',v)

    vprint('setting pwms to default values ... ',v, comma=True)
    dev.set_pwm_to_default()
    vprint('done',v)
           
    # Close device
    vprint('closing device ... ', v, comma=True)
    dev.close()
    vprint('done',v)
    return

def reset(options):
    v = options.verbose  
    # Open device
    vprint('opening device ... ',v,comma=True)
    dev = clkdirpwm_device()
    vprint('done',v)

    vprint('reseting ... ',v, comma=True)
    dev.reset()
    vprint('done',v)
           
    # Close device
    vprint('closing device ... ', v, comma=True)
    dev.close()
    vprint('done',v)
    return

def read(options):
    v = options.verbose  
    # Open device
    vprint('opening device ... ',v,comma=True)
    dev = clkdirpwm_device()
    vprint('done',v)

    # Send test command
    vprint('sending test command ... ',v, comma=True)
    dev.read()
    vprint('done',v)
           
    # Close device
    vprint('closing device ... ', v, comma=True)
    dev.close()
    vprint('done',v)
    return

def test(options):
    v = options.verbose  
    # Open device
    vprint('opening device ... ',v,comma=True)
    dev = clkdirpwm_device()
    vprint('done',v)

    # Send test command
    vprint('sending test command ... ',v, comma=True)
    dev.test()
    vprint('done',v)
           
    # Close device
    vprint('closing device ... ', v, comma=True)
    dev.close()
    vprint('done',v)
    return

def vprint(msg, verbose, comma=False):
    """ Print statement for verbose mode"""
    if verbose==True:
        if comma==False or DEBUG==True:
            print msg
            sys.stdout.flush()
        else:
            print msg, 
            sys.stdout.flush()

# -------------------------------------------------------------------------
if __name__=='__main__':
    clkdirpwm_main()

