package it.quadrotorcommander;

import android.os.Bundle;
import android.app.Activity;
import android.content.Intent;
import android.view.Menu;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;


public class MainActivity extends Activity{

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		System.out.println("MAIN CREATE");
		setContentView(R.layout.activity_main);
		
		Button startButton = (Button) findViewById(R.id.button1);
		startButton.setOnClickListener(new View.OnClickListener() {
	        
        	//@Override
        	public void onClick(View view)
        	{
        		final String address = ((TextView) findViewById(R.id.user_input)).getText().toString();
        		Intent speech = new Intent(MainActivity.this, SpeechActivity.class);
        		speech.putExtra("EXTRA_ADDRESS", address);
        		//System.out.println(address+"<:-)");
        		startActivity(speech);
        		finish();
        		//startService(lcm);
        	}	
        }
		);
		
		/*try{
			lcm = new LCM("tcpq://"+host+":7700");
			lcm.subscribe("CONTROL", this);
		}catch(Exception e){
			System.out.println(e);
		}*/
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.activity_main, menu);
		return true;
	}
	
	/*public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        System.out.println("Received message on channel " + channel);
    }*/

}
