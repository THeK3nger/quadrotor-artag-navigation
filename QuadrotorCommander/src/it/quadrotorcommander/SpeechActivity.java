package it.quadrotorcommander;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import lcmtypes.Speech;
import lcmtypes.SpeechList;

import it.quadrotorcommander.LcmService.LocalBinder;
import android.os.Bundle;
import android.os.IBinder;
import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import android.speech.RecognizerIntent;
import android.speech.tts.TextToSpeech;
import android.speech.tts.TextToSpeech.OnInitListener;
import android.view.Menu;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ListView;

public class SpeechActivity extends Activity implements OnClickListener, OnInitListener {
	
	private String host;
	LcmService lcmService;
    boolean lcmBound = false;
    
    private static final int VOICE_RECOGNITION_REQUEST_CODE = 1234;
	 // Definition of the one requestCode we use for receiving results.
    private static final int RESULTS_REQUEST_CODE = 1337;
	private ListView mList;
	private TextToSpeech mTts;
	
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
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_speech);
		
		host=this.getIntent().getStringExtra("EXTRA_ADDRESS");
		
		Intent lcm = new Intent(SpeechActivity.this, LcmService.class);
		lcm.putExtra("EXTRA_ADDRESS", host);
		startService(lcm);
		
		// Get display items for later interaction
        Button speakButton = (Button) findViewById(R.id.button1);

        mList = new ListView(this);//(ListView) findViewById(R.id.list);

        // Check to see if a recognition activity is present
        PackageManager pm = getPackageManager();
        List<ResolveInfo> activities = pm.queryIntentActivities(
                new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH), 0);
        if (activities.size() != 0) {
            speakButton.setOnClickListener(this);
        } else {
            speakButton.setEnabled(false);
            speakButton.setText("Recognizer not present");
        }
        
        // Initialize text-to-speech. This is an asynchronous operation.
        // The OnInitListener (second argument) is called after initialization completes.
        mTts = new TextToSpeech(this, this);
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.activity_speech, menu);		
		return true;
	}
	
	protected void onStart() {
        super.onStart();
        // Bind to LocalService
        Intent intent = new Intent(this, LcmService.class);
        bindService(intent, mConnection, Context.BIND_AUTO_CREATE);
    }
	
	protected void onStop(){
		super.onStop();
		if (lcmBound){
		 unbindService(mConnection);
		 lcmBound=false;
		 stopService(new Intent(this, LcmService.class));
		}
		//CONTROLLARE SE QUI O ON DESTROY
		if (mTts != null) {
            mTts.stop();
            mTts.shutdown();
        }
	}
	
	protected void onResume() {
        super.onResume();
        Intent intent = new Intent(this, LcmService.class);
        bindService(intent, mConnection, Context.BIND_AUTO_CREATE);
    }
	
	protected void onPause() {
		super.onPause();
		if (lcmBound){
			 unbindService(mConnection);
			 lcmBound=false;
		}
	}
	
    public void onUtteranceCompleted(String uttId) {
    	System.out.println("done with speech");
        /*TTSStatus tts_status = new TTSStatus();
        tts_status.timestamp = System.currentTimeMillis() / 1000L;
        tts_status.finished = true;
        
    	lcm.publish("TTSStatus", tts_status);*/
    }
    
    public void onClick(View v) {
    	System.out.println("Click!!");
        if (v.getId() == R.id.button1) {
        	System.out.println("Speak!");
            startVoiceRecognitionActivity();
        }
    }

    /**import lcm.lcm.LCM;
     * Fire an intent to start the speech recognition activity.
     **/
    private void startVoiceRecognitionActivity() {
        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL,
                		RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        intent.putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 5);
        intent.putExtra(RecognizerIntent.EXTRA_PROMPT, "Tell the robot what to do.");
        intent.putExtra(RecognizerIntent.EXTRA_CONFIDENCE_SCORES, 5);
        startActivityForResult(intent, VOICE_RECOGNITION_REQUEST_CODE);
    }

    /**
     * Handle the results from the recognition activity.
     */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (requestCode == VOICE_RECOGNITION_REQUEST_CODE && resultCode == RESULT_OK) {
            // Fill the list view with the strings the recognizer thought it could have heard
            ArrayList<String> matches = data.getStringArrayListExtra(
                    RecognizerIntent.EXTRA_RESULTS);
            float[] confidence = data.getFloatArrayExtra(RecognizerIntent.EXTRA_CONFIDENCE_SCORES);
           
            mList.setAdapter(new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1,
                    matches));
            
            
            //SOSTITUIRE CON CHIAMATA SERVICE
           
            //create an array that contains the speech results
            Speech[] speechArray = new Speech[matches.size()];
            
            for(int i=0; i<matches.size(); i++){
        		Speech rec = new Speech();
        		rec.text = matches.get(i);
        		rec.timestamp = System.currentTimeMillis() / 1000L;
        		//System.out.println("CONFIDENCE: "+confidence[i]);
        		rec.confidence = confidence[i];
        		speechArray[i] = rec;
            }
            
            //make an LCM message out of the results
            SpeechList resultsList = new SpeechList();
            resultsList.length = matches.size();
            resultsList.results = speechArray;
            
            System.out.println("Publishing speech");
            lcmService.publishMessage("SPEECH", resultsList);
            //send the results over the network
//            lcm.publish("SPEECH", resultsList);
//            System.out.println("ok!!");
        }
        else if(requestCode == RESULTS_REQUEST_CODE && resultCode == RESULT_OK){
        	
        	System.out.println(data.getAction());
        	
            /*Response msg = new Response();
            msg.timestamp = System.currentTimeMillis() / 1000L;
            msg.question = question_current;
            msg.response = data.getAction();
            lcm.publish("RESPONSE", msg);*/
        }
        
        
        //send the recognized speech to be processed
        super.onActivityResult(requestCode, resultCode, data);
    }

    public void onInit(int status){
    	mTts.setLanguage(Locale.US);
    }

}
