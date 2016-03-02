# -*- coding: utf-8 -*-
"""
Created on Wed Sep 30 12:30:49 2015

@author: shrevz
"""
import cmd, sys, numpy, udsp
class CLI(cmd.Cmd):
    def __init__(self):
        cmd.Cmd.__init__(self)
        self.p = udsp.UDSP()
        self.di = None
        
    def do_serve(self, addr):
        """
        Set up server socket at specified path
        
        Default is /tmp/UDS_socket
        """
        addr=addr.strip()
        if not addr:
            addr = '/tmp/UDS_socket'
        self.p.serveAt(addr)
        self.di = self.p.dataIter()
    
    def do_rx(self,_):
        """
        Receive an object via socket
        """
        print repr(self.di.next())
    
    def do_close(self,_):
        """
        Close a connection
        """
        self.p.close()
    
    def do_connect(self, addr):
        """
        Connect to specified address 
        
        Default is /tmp/UDS_socket
        """
        addr = addr.strip()
        if not addr:
            addr = '/tmp/UDS_socket'
        self.p.connect(addr)
        self.di = self.p.dataIter()
        
    def do_tx(self, expr):
        """
        Transmit a python object to the peer
        Receive it using the rx command
        
        NOTE: the numpy module is in the namespace; you can
        use expressions that use numpy arrays
        """
        obj = eval(expr.strip())
        print "Sending: ",repr(obj)
        self.p.sendObj(obj)
    
    def do_EOF(self,_):
        self.p.close()
        return True

    def do_exampleServer(self,_):
        """
        An example of a complete server loop
        """
        p = udsp.UDSP()
        p.serveAt('/tmp/example')
        di = p.dataIter()
        print "Ready for data..."
        for obj in di:
            print '-'*40
            print repr(obj)
        print "...done"
        p.close()
    
    def do_exampleClient(self,_):
        """
        An example of a complete client loop
        """
        p = udsp.UDSP()
        p.connect('/tmp/example')
        print "Connected..."
        while True:
            obj = input("expr / None to quit> ")
            if obj is None:
                break
            p.sendObj(obj)
        print "...closing"
        p.close()
        
if __name__=="__main__":
    cli = CLI()
    cli.cmdloop()