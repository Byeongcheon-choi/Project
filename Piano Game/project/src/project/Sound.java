package project;

import java.io.IOException;

import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.UnsupportedAudioFileException;
import java.io.File;

public class Sound extends Thread{
	
	private Clip clip;
	private boolean loop;
	private File audioFile;
	private	AudioInputStream audioStream;
	private String pathName;
	
	public Sound(String path, boolean isLoop)
	{
		try {
			loop = isLoop; 
			pathName = path;
			clip = AudioSystem.getClip();
			audioFile = new File(pathName);
			AudioInputStream audioStream = AudioSystem.getAudioInputStream(audioFile);
			clip.open(audioStream);
		} catch (LineUnavailableException e) {
			e.printStackTrace();
		} catch (UnsupportedAudioFileException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public int getTime() {  // This is a method for catching the block.
		if(clip == null)
		{
			return 0;
		}
		return clip.getFramePosition();
	}
	public void close() {
		loop = false;
		clip.stop();
		this.interrupt();
	}
	
	public void run() {
		try {
			clip = AudioSystem.getClip();
			audioFile = new File(pathName);
			AudioInputStream audioStream = AudioSystem.getAudioInputStream(audioFile);
			clip.open(audioStream);
			clip.start();
			if (loop)
				clip.loop(Clip.LOOP_CONTINUOUSLY);			
		}
		catch(Exception e) {
			System.out.println(e.getMessage());
		}
	}
	
}


