# -*- coding: utf-8 -*-
"""
Created on Wed Sep 30 11:21:16 2015

@author: shrevz
"""

from socket import socket, AF_UNIX, SOCK_STREAM
from os import getpid, unlink
from os.path import exists as os_path_exists
from cPickle import loads, dumps, HIGHEST_PROTOCOL

class UDSP(object):
    def __init__(self,addr=None):
        self.sock = None 
        self.conn = None
        self.addr = None
        if addr is not None:
            return self.serveAt(addr)
        
    def serveAt(self, addr):
        """
        Open a server socket at specified location and wait for a connection
        """
        assert self.sock is None
        # Make sure the socket does not already exist
        try:
            unlink(addr)
        except OSError:
            if os_path_exists(addr):
                raise
        # Create a UDS socket
        self.sock = socket(AF_UNIX, SOCK_STREAM)
        # Bind the socket to the port
        self.sock.bind(addr)
        self.addr = addr
        self.sock.listen(1)
        conn, caddr = self.sock.accept()
        self.conn = conn
        return
    
    def dataIter(self):
        """
        Iterator yielding incoming pickled objects from the 
        specified connection.
        """
        conn = self.conn
        while True:
            # Receive hex encoded object length
            hdr = conn.recv(16)
            # Empty message indicates end of connection
            if not hdr:
                break
            sz = int(hdr,16)         
            # Receive sz bytes
            buf = []
            while sz>0:
                buf.append(conn.recv(min(sz,4096)))
                sz -= len(buf[-1])
            # Un-pickle
            obj = loads("".join(buf))
            yield obj
    
    def connect( self, addr ):
        """
        Connect to specified address
        """
        assert self.sock is None
        # Create a UDS socket
        self.sock = socket(AF_UNIX, SOCK_STREAM)
        self.sock.connect(addr)
        self.conn = self.sock
        
    def close(self):
        """
        Close a connection returned by connIter()
        """
        if self.conn is not None:
            self.conn.close()
        # if this was a server socket at self.addr --> unlink
        if self.addr is not None:
            self.sock.close()
            unlink(self.addr)
        self.sock = None
        self.conn = None
        self.addr = None
        
    def sendObj( self, obj ):
        """
        Send an object through a connection
        """
        conn = self.conn        
        msg = dumps(obj,HIGHEST_PROTOCOL)
        sz = "%016x" % len(msg)
        # Send hex encoded object length
        conn.send(sz)
        # Send object
        conn.sendall(msg)


