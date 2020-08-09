package test;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.IOException;

import javax.sound.sampled.AudioInputStream;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.Clip;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.UnsupportedAudioFileException;
import javax.swing.ImageIcon;
import javax.swing.JFrame;

public class Day1 extends JFrame{
	
	private Image backgroundImage = new ImageIcon("src/test/images/Backpicture.jpg").getImage();
	private Image pixel = new ImageIcon("src/test/images/pixel.jpg").getImage();
	private Image coin = new ImageIcon("src/test/images/coin.png").getImage();
	private Image bufferImage; // 
	private Graphics screenGraphics;
	
	private Clip clip;
	
	private boolean up,down,left,right;
	
	private int pixel1, pixel2;
	private int coinX, coinY;
	
	private int playerWidth = pixel.getWidth(null);
	private int playerHeight = pixel.getHeight(null);	
	private int coinWidth = coin.getWidth(null);
	private int coinHeight = coin.getHeight(null);
	
	private int score;
	
	
	public static void main(String[] args) {
		new Day1();
	}
	
	public Day1() {
		setTitle("Pixel Game");
		setVisible(true);
		setSize(700,700);
		setLocationRelativeTo(null);
		setResizable(false); 
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	
		init(); // set the player and coin position and background sound.
		
		addKeyListener(						// Detecting Key value
		new KeyAdapter()
			{          
				public void keyPressed(KeyEvent e) 
				{
					switch(e.getKeyCode()) {
					case KeyEvent.VK_UP:
						up = true;
						break;
					case KeyEvent.VK_DOWN:
						down = true;
						break;
					case KeyEvent.VK_LEFT:
						left = true;
						break;
					case KeyEvent.VK_RIGHT:
						right = true;
						break;
					}
				}
				
				public void keyReleased(KeyEvent e) 
				{
					switch(e.getKeyCode()) {
					case KeyEvent.VK_UP:
						up = false;
						break;
					case KeyEvent.VK_DOWN:
						down = false;
						break;
					case KeyEvent.VK_LEFT:
						left = false;
						break;
					case KeyEvent.VK_RIGHT:
						right = false;
						break;
					}
				}
			}
		);
		
		while(true) {
			try {
				Thread.sleep(20);
			}catch(InterruptedException e) {
				e.printStackTrace();
			}

			keyProcess();
			crashChek();
		
		}
	}
	
	private void init() {  //set the Score, pixel, coin position in the screen.
		this.score = 0;	
		pixel1 = (700 - playerWidth)/2;
		pixel2 = (700 - playerHeight)/2;
		boolean Limit = true;			//Create the coin avoid the block area.
		do {
			coinX = (int)(Math.random()*(701-playerWidth));
			coinY = (int)(Math.random()*(701-playerHeight-30))+30;
			if(coinX > 400 - coinWidth/2 && coinX < 600 && coinY > 550) {
				System.out.println(coinX + " 1 "+ coinY);
				continue;
			}
			else if(coinX > 400 - (coinWidth/2) && coinX < 600 && coinY < 300  && coinY > 150) {
				System.out.println(coinX + " 2 "+ coinY);
				continue;
			}
			else if(coinX > 200 - (coinWidth/2) && coinX < 300  && coinY < 400 && coinY > 50){
				System.out.println(coinX + " 3 "+ coinY);
				continue;
			}
			else if(coinX > 100 - (coinWidth/2) && coinX < 400&& coinY < 600  && coinY > 450) {
				System.out.println(coinX + " 4 "+ coinY);
				continue;
			}
			Limit = false;
		}while(Limit);

		playSound("src/test/sound/backgroundMusic.wav", true);
	}
	
	public void keyProcess(){  // By using key value, determine how pixel should be changed.
		if (up && pixel2 - 3 > 30) pixel2-=3;	
		if (down && pixel2 + playerHeight + 3 < 700) pixel2+=3;	
		if (left && pixel1 - 3 > 0) pixel1-=3;
		if (right && pixel1 + playerWidth + 3 < 700) pixel1+=3;

		if(BlockTest(pixel1,pixel2) == false)
		{
			pixel1 = (700 - playerWidth)/2;
			pixel2 = (700 - playerHeight)/2;
			System.out.println("dign");
		}
	}
	
	
	public Boolean BlockTest(int a, int b) // When colliding with block, go back to the original point
	{
		boolean A,B,C,D;
		A=true;
		B=true;
		C=true;
		D=true;
		
		if(pixel1 > 400 - playerWidth/2 && pixel1 < 600 && pixel2 > 550) {
			System.out.println("¸ÓÂ¡");
			A=false;
		}
		else if(pixel1 > 400 - playerWidth/2  && pixel1 < 600  && pixel2 < 300  && pixel2 > 150) {
			System.out.println("¸ÓÂ¡");
			B=false;
		}
		else if(pixel1 > 200 - playerWidth/2 && pixel1 < 300 && pixel2 < 400 && pixel2 > 50){
			
			C=false;
		}
		else if(pixel1 > 100 - playerWidth/2 && pixel1 < 400 && pixel2 < 600 && pixel2 > 450) {
			System.out.println("¸ÓÂ¡");
			D=false;
		}
		if(A&B&C&D)
		{
			return true;
		}
		else
			return false;
	}
	
	
	public void crashChek() {   // the method of colliding between coin and pixel
		if(pixel1 + playerWidth > coinX && coinX + coinWidth > pixel1 && pixel2 + playerHeight > coinY && coinY + coinHeight > pixel2) {
			
			this.score+=100;
			boolean Limit = true;
			do {
				coinX = (int)(Math.random()*(701-playerWidth));
				coinY = (int)(Math.random()*(701-playerHeight-30))+30;
				if(coinX > 400 - coinWidth/2 && coinX < 600 && coinY > 550) {
					System.out.println(coinX + " 1 "+ coinY);
					continue;
				}
				else if(coinX > 400 - (coinWidth/2) && coinX < 600 && coinY < 300  && coinY > 150) {
					System.out.println(coinX + " 2 "+ coinY);
					continue;
				}
				else if(coinX > 200 - (coinWidth/2) && coinX < 300  && coinY < 400 && coinY > 50){
					System.out.println(coinX + " 3 "+ coinY);
					continue;
				}
				else if(coinX > 100 - (coinWidth/2) && coinX < 400&& coinY < 600  && coinY > 450) {
					System.out.println(coinX + " 4 "+ coinY);
					continue;
				}
				Limit = false;
			}while(Limit);
			System.out.println(coinX + " result "+ coinY);
			System.out.println(coinWidth);
			playSound("src/test/sound/soundEffect.wav", false);
		}
	}
	
	public void screenDraw(Graphics g) {  // set the Score, pixel, coin position in the screen.
		g.drawImage(backgroundImage,0,0,null);
		g.drawImage(pixel,pixel1,pixel2,null);
		g.drawImage(coin,coinX,coinY,null);
		g.setColor(Color.red);
		g.setFont(new Font("Arial", Font.BOLD, 40));
		g.drawString("Score : " + score, 30, 80);
		
        g.setColor( new Color(255,0,0) ); // Make a obstacle.    
        g.fillRect(400,600,200,100);
        g.fillRect(400,200,200,100);
        g.fillRect(200,100,100,300);
        g.fillRect(100,500,300,100);

        
		this.repaint();
	}
	
	public void paint(Graphics g) {         //prevent the screen buffer. 
		bufferImage = createImage(700, 700);
		screenGraphics = bufferImage.getGraphics();
		screenDraw(screenGraphics);
		g.drawImage(bufferImage, 0, 0, null);
	}
	
	public void playSound(String pathName, boolean isLoop) { //set the background sound.
		try {
			clip = AudioSystem.getClip();
			File audioFile = new File(pathName);
			AudioInputStream audioStream = AudioSystem.getAudioInputStream(audioFile);
			clip.open(audioStream);
			clip.start();
			if (isLoop)
				clip.loop(Clip.LOOP_CONTINUOUSLY);
		} catch (LineUnavailableException e) {
			e.printStackTrace();
		} catch (UnsupportedAudioFileException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
