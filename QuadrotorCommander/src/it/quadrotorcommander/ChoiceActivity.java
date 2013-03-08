package it.quadrotorcommander;

import it.quadrotorcommander.LcmService.LocalBinder;
import android.os.Bundle;
import android.os.IBinder;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.view.Menu;
import android.view.View;
import android.widget.Button;
import android.widget.RadioGroup;

public class ChoiceActivity extends Activity {
	
	LcmService lcmService;
    boolean lcmBound = false;
	
	private ServiceConnection mConnection = new ServiceConnection() {

        public void onServiceConnected(ComponentName className, IBinder service) {
            // We've bound to LocalService, cast the IBinder and get LocalService instance
            LocalBinder binder = (LocalBinder) service;
            lcmService = binder.getService();
            lcmBound = true;
        }

        public void onServiceDisconnected(ComponentName arg0) {
            lcmBound = false;
        }
    };

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		//if (savedInstanceState == null){
			//System.out.println("AIUTO");
		//}else{
			super.onCreate(savedInstanceState);
		//}
		setContentView(R.layout.activity_choice);
		
		final RadioGroup options = (RadioGroup) findViewById(R.id.radioGroup1);
		Button startButton = (Button) findViewById(R.id.buttonChoice);
		startButton.setOnClickListener(new View.OnClickListener() {
	        
        	//@Override
        	public void onClick(View view)
        	{
        		int choice = options.getCheckedRadioButtonId();
        		System.out.println(choice);
        		if(lcmBound){
        			switch (choice) {
        				case R.id.radio0 : lcmService.publishMessage("CHOICE", "FORWARD");
        					break;
        				case R.id.radio1 : lcmService.publishMessage("CHOICE", "BACKWARD");
        					break;
        				case R.id.radio2 :lcmService.publishMessage("CHOICE", "STOP");
        					break;
        				case R.id.radio3 :lcmService.publishMessage("CHOICE", "BEGIN");
    						break;
        				case R.id.radio4 :lcmService.publishMessage("CHOICE", "END");
    						break;
        			}
        			finish();
        		}
        	}	
        }
		);
	}
	
	@Override
	public void onBackPressed(){
		//muahahah
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.activity_sciocca, menu);
		return true;
	}
	
	@Override
    protected void onStart() {
        super.onStart();
        // Bind to LocalService
        Intent intent = new Intent(this, LcmService.class);
        bindService(intent, mConnection, Context.BIND_AUTO_CREATE);
    }
	
	public void onStop(){
		super.onStop();
		if (lcmBound){
		 unbindService(mConnection);
		 lcmBound=false;
		}
	}

}
