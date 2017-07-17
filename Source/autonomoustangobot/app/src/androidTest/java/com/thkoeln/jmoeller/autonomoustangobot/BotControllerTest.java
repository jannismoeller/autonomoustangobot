package com.thkoeln.jmoeller.autonomoustangobot;

import android.support.test.runner.AndroidJUnit4;

import com.thkoeln.jmoeller.autonomoustangobot.botcontroll.BotController;

import org.junit.Test;
import org.junit.runner.RunWith;

import static android.os.SystemClock.sleep;
import static org.junit.Assert.*;

/**
 * Test-Class for the BotController
 * Use with caution! Physical Testing, the Bot will move
 */
@RunWith(AndroidJUnit4.class)
public class BotControllerTest {
    @Test
    public void testBotConnection(){
        BotController botController = new BotController();

        botController.connect("192.168.43.48");

        sleep(2000);

        assertTrue(botController.isConnected());

        botController.forward(20);
        sleep(500);
        botController.forward(50);
        sleep(500);
        botController.forward(100);
        sleep(500);
        botController.backward(150);
        sleep(500);
        botController.backward(300);
        sleep(500);
        botController.forward(360);
        sleep(2000);

        botController.rotateLeft(300);
        sleep(500);
        botController.rotateLeft(150);
        sleep(500);
        botController.rotateLeft(100);
        sleep(500);
        botController.rotateLeft(50);
        sleep(500);
        botController.rotateRight(50);
        sleep(500);
        botController.rotateRight(20);
        sleep(500);
        botController.rotateRight(10);
        sleep(500);
        botController.stop();
        sleep(2000);

        botController.disconnect();
        sleep(2000);
    }
}
