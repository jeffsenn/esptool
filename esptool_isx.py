from __future__ import division, print_function
import sys
import time

#class that uses module property "^" to communicate with ESP32 mounted on ISX
class ISXSerial:
    def __init__(self,arg):
        # split args to get UUIDs valid for programming
        self.module_uuids = arg.upper().split(",")
        self.dtr = 0
        self.inbuf = ''
        self.outbuf = ''
        self.timeout = 10
        self.noseqnum = False
        self.syncbuf = False
        self.reseti()
        self.closed = False

    def close(self):
        self.closed = True
        
    def reseti(self):
        self.in_state = {}
        self.iesc = 0
        self.module = 0
        self.selected = {}
        self.pending_write = False
        
    def escapeff(self,s):
        x = s.split("\xfe")
        ret = []
        for a in x:
            a = a.replace("\xff","\xfe\xfe")
            a = a.replace("\xfd","\xfe\xfc")
            ret.append(a)
        return "\xfe\xfd".join(ret)
        
    def open(self):
        import usb.core,usb.util
        from MAYA.VIA import vsmf
        self.found = None
        ISX_VID=0x0925
        ISX_PID=0x9099
        ret = []
        self.usb = usb
        self.vsmf = vsmf
        dev = usb.core.find(find_all=True, idVendor=ISX_VID, idProduct=ISX_PID)
        for d in dev:
            try:
                try:
                    ret.append((usb.util.get_string(d,256,3),d))
                except:
                    ret.append((usb.util.get_string(d,3),d))
            except:
                print ("Can't query device",d)
        b = ret[0][1]
        b.set_configuration()
        cfg = b.get_active_configuration()
        interface_number = cfg[(0,0)].bInterfaceNumber
        #alternate_settting = usb.control.get_interface(interface_number)
        intf = usb.util.find_descriptor(
            cfg, bInterfaceNumber = interface_number,
            #bAlternateSetting = alternate_setting
            )
        self.oep = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = \
            lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_OUT
            )
        self.iep = usb.util.find_descriptor(
            intf,
            # match the first OUT endpoint
            custom_match = \
            lambda e: \
            usb.util.endpoint_direction(e.bEndpointAddress) == \
            usb.util.ENDPOINT_IN
            )

        import thread
        thread.start_new_thread(self.read_thread,())
        self.openstack()

    def openstack(self):
        # sync - and reset
        self.outbuf = "\xfe\xfb\x04Isx?\xff"
        self.stackstate = 0
        while self.stackstate == 0: # wait for reset/module detect
            self.write_some()
            time.sleep(0.1)
        print("---ISX:FOUND MODULE",self.found,"---")

    # return (or create) selected terms list for m
    def get_selected(self,m):
        x = self.selected.get(m)
        if x is None:
            x = self.selected[m] = [0]
        return x

    def in_filter(self, x):
        for c in x:
            oc = ord(c)
            if oc == 255:
                print("---ISX-RESET---")
                self.reseti()
                continue
            elif self.iesc == 254:
                self.iesc = 0
                if oc < 128:
                    self.module = oc
                    continue
                elif oc > 251:
                    oc += 1
                    c = chr(oc)
                elif oc == 251:
                    self.iesc = 2000
                    continue
                elif oc == 250:
                    oc = -1
                else:
                    raise Exception("protocol violation")
            elif self.iesc == 253:
                self.iesc = 0
                if oc < 128:
                    self.get_selected(self.module)[:] = [oc]
                elif oc < 254:
                    self.get_selected(self.module).append(oc-128)
                else:
                    self.iesc = 1000 # special esc
                continue
            elif self.iesc == 1000:
                self.iesc = 0
                self.get_selected(self.module).append(oc-252+125)
                continue
            elif self.iesc == 2000:
                self.oob_len = oc
                if oc > 0:
                    self.iesc = 2001
                    self.oob = ''
                else:
                    self.iesc = 0
                continue
            elif self.iesc == 2001:
                self.oob += c
                self.oob_len -= 1
                if self.oob_len == 0:
                    #print("OOB:",repr(self.oob))
                    if self.oob == "Isx01":
                        #print("OOB packet indicates ISX01 - switching to noseqnum mode")
                        self.noseqnum = True
                        self.syncbuf = True
                    self.iesc = 0
                continue
            elif oc == 254 or oc == 253:
                self.iesc = oc
                continue
            # if you got here you're reading a msg to self.module/self.selected[self.module]
            ky = str(self.module)+":"+'-'.join(map(str,self.get_selected(self.module)))
            st = self.in_state.get(ky)
            #if DEBUG: print("]]",ky,self.iesc,oc,st)
            if oc == -1: # EOM
                if st is not None:
                    self.in_state.pop(ky)
                    self.handle_message(ky,st[1])
                #else empty msg?
            else:
                if st is None:
                    st = self.in_state[ky] = [0,'']
                if self.syncbuf:
                    self.syncbuf = False
                    st[1] = ''
                    st[0] = 0
                st[1] += c
                st[0] += 1

    def sendstack(self, msg):
        self.outbuf += self.escapeff(self.vsmf.serialize(msg))+"\xfe\xfa"
        self.write_some()

    def write_more(self): # write more from buffered stuff
        while not self.pending_write and len(self.towrite):
            self.pending_write = True
            l = min(32,len(self.towrite))
            x = self.towrite[:l]
            self.towrite = self.towrite[l:]
            self.sendstack(["^",self.vsmf.Binary(x)])
        
    def got_bin(self,b):
        if len(b) == 0: # ack from module
            self.pending_write = False
            self.write_more()
        else:
            self.inbuf += b
            
    def handle_message(self,ky,rmsg):
        msg = self.vsmf.reconstitute(rmsg)
        #print("MSG",ky,msg, repr(rmsg))
        if self.stackstate == 0: # wait for stack enum
            if ky.endswith(":0") and type(msg) == type([]) and len(msg) == 2 and msg[0] == 'T':
                print("looking",msg[1].toString(), self.module_uuids)
                if msg[1].toString().upper() in self.module_uuids:
                    self.found = int(ky[0]) # found the module!
            if msg == []:
                self.outbuf += "\xfe"+chr(self.found) # select module
                self.outbuf += "\xfd\x00" # term 0
                self.sendstack(['^',True])
            if msg == True and ky == str(self.found)+":0":
                self.stackstate = 1 # ready to go
        elif self.stackstate == 1:
            if type(msg) == type([]) and len(msg) == 2:
                if msg[0] == '^':
                    self.got_bin(msg[1])
                    

    def read_thread(self):
      while not self.closed:
        ret = None
        try:
            ret = self.iep.read(64,10000000) # functionally infinite timeout
        except self.usb.core.USBError as e:
            if e.args == (60,'Operation timed out',):
                return
            if e.args == (32,'Pipe error',):
                print("ISX Base closed")
                sys.exit(1)
                print (repr(e.args))
                raise
        if ret == None:
          print("BASE CLOSED")
          sys.exit(1)
        rd = ''.join([chr(x) for x in ret])
        #print("READ",repr(rd))
        self.in_filter(rd)
        
    def write_some(self):
        while len(self.outbuf) > 0:
            x = self.outbuf[:64]
            self.outbuf = self.outbuf[64:]
            try:
                    #print("WRITING",repr(x))
                    ret = self.oep.write(x,1)
                    #print("WROTE",ret)
                    if ret != len(x):
                        print("Possible short write?",repr(x),ret,len(x))
                    break
            except self.usb.core.USBError as e:
                    print("Error",repr(e))
                    if repr(e) == "USBError(32, 'Pipe error')":
                        print ("Stall...")
                        time.sleep(0.1)
                        break
                    raise
        
        
    def inWaiting(self):
        return len(self.inbuf)
    def read(self,amt):
        #print(">RD",amt,self.timeout)
        i = 0
        while len(self.inbuf) < amt:
            time.sleep(0.1)
            i += 1
            if i > 200:
                break
        amt = min(amt,len(self.inbuf))
        ret = self.inbuf[:amt]
        self.inbuf = self.inbuf[amt:]
        #print("<RD",repr(ret))
        return ret
    def write(self,buf):
        #print(">WR",repr(buf))
        self.towrite += buf
        self.write_more()
        #print("<WR",len(self.towrite))
    def flushOutput(self):
        self.towrite = ''
    def flushInput(self):
        self.inbuf = ''
    def setDTR(self,x):
        pass
    def setRTS(self,x):
        pass

def patch_serial_for_url(module):
    def serial_for_url(x):
        #print ("URL SERIAL", x)
        if x.startswith("ISX://"):
            ret = ISXSerial(x[6:])
            ret.open()
            return ret
        return base_serial_for_url(x)
    base_serial_for_url = module.serial_for_url
    module.serial_for_url = serial_for_url

