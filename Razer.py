#!/usr/bin/python

import yarp

yarp.Network.init()

p = yarp.BufferedPortBottle()
p.open("/python");

yarp.Network.connect("/razer", "/python")

cont = True
while cont:
    
    btl = p.read(True)

    #Get position of Controller #1
    print("\n"+"Pos X1: "+ str(btl.get(0).asDouble())),
    print("Pos Y1: "+ str(btl.get(1).asDouble())),
    print("Pos Z1: "+str(btl.get(2).asDouble()))
    
    #Get rotation matrix of Controller #1
    print ("Rot Matrix Controller 1")
    print(""+ str(btl.get(3).asDouble())),
    print("  "+ str(btl.get(4).asDouble())),
    print("  "+str(btl.get(5).asDouble()))

    print(""+ str(btl.get(6).asDouble())),
    print("  "+ str(btl.get(7).asDouble())),
    print("  "+str(btl.get(8).asDouble()))

    print(""+ str(btl.get(9).asDouble())),
    print("  "+ str(btl.get(10).asDouble())),
    print("  "+str(btl.get(11).asDouble()))


    #Get position of Controller #2
    print("\n"+"Pos X2: "+ str(btl.get(12).asDouble())),
    print("Pos Y2: "+ str(btl.get(13).asDouble())),
    print("Pos Z2: "+str(btl.get(14).asDouble()))
    
    #Get rotation matrix of Controller #1
    print ("Rot Matrix Controller 2")
    print(""+ str(btl.get(15).asDouble())),
    print("  "+ str(btl.get(16).asDouble())),
    print("  "+str(btl.get(17).asDouble()))

    print(""+ str(btl.get(18).asDouble())),
    print("  "+ str(btl.get(19).asDouble())),
    print("  "+str(btl.get(20).asDouble()))

    print(""+ str(btl.get(21).asDouble())),
    print("  "+ str(btl.get(22).asDouble())),
    print("  "+str(btl.get(23).asDouble()))
    
    yarp.Time.delay(0.1)

p.close();

yarp.Network.fini();
