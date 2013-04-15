//package slu.lcm.LCMServer;

import lcm.lcm.*;
import java.io.IOException;

/// <summary>
/// Simple TCP provider server implementation
/// </summary>
public class LCMServer{
    public static void main(String args[]){

	try{
	    int port = 7700;
	    if (args.length > 0)
		{
		    port = Integer.parseInt(args[0]);
		}
	    System.out.println("Connected to port " + port);
	    new TCPService(port);
	}
	catch (IOException e){
	    System.out.println("Ex: " + e);
	}
    }
}
