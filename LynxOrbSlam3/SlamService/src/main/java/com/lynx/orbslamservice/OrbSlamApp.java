package com.lynx.orbslamservice;

import android.app.Activity;
import android.content.Intent;
import android.content.res.AssetManager;
import android.os.Bundle;
import android.util.Log;
import android.view.KeyEvent;

import androidx.appcompat.app.AppCompatActivity;

public class OrbSlamApp extends AppCompatActivity {


    private static final String TAG = "OrbSlamApp (JAVA)";
    static {
        System.loadLibrary("orbslam_service");
    }


    native void startSlamSystem(String nativeLibPath, String internalDataPath, AssetManager assetManager);

    native void stopSlamSystem();

    @Override
    protected void onResume() {
        super.onResume();

        Log.d("ORBSLAM3 ServiceLauncher", "Starting orbslam3 service...");

        Log.d(TAG, "ORB-SLAM3 App Launching...");

        // Start the ORB-SLAM3 system
        startSlamSystem(getApplicationInfo().nativeLibraryDir + "/", getApplicationInfo().dataDir, getAssets());

        Log.d(TAG, "ORB-SLAM3 App Launched...");


        try {
            Thread.sleep(1000); // Wait for a second to ensure the service is ready
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (event.getAction() == KeyEvent.ACTION_DOWN) {
            if (keyCode == KeyEvent.KEYCODE_SOFT_RIGHT) {
                finish();
                return true; // consommé
            }
        }
        return super.onKeyDown(keyCode, event);
    }

    @Override
    protected void onStop() {
        super.onStop();

        Log.d(TAG, "ORB-SLAM3 App Stopping...");

        // Stop the ORB-SLAM3 system
        stopSlamSystem();
    }

}
