package project;
import java.awt.Graphics;
import java.awt.Image;

import javax.swing.ImageIcon;
import javax.swing.JFrame;


public class UI extends JFrame{
		
		public static final int SCREEN_WIDTH =1280;
		public static final int SCREEN_HEIGHT = 720;
		private Image screenImage;
		private Graphics screenGraphic;
		
		private Image introBackground;
		
		public UI()
		{
			setTitle("Piano Game");
			setSize(Main.SCREEN_WIDTH,Main.SCREEN_HEIGHT);
			setResizable(false);
			setLocationRelativeTo(null);
			setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			setVisible(true);
			
			introBackground = new ImageIcon(Main.class.getResource("../Backeffect/backback.jpg")).getImage();
			Sound s= new Sound("src/sound/SOUND1.wav",true); // Start Sound
			s.start();										//Thread setting.
		}
		
		public void paint(Graphics g)
		{
			screenImage = createImage(Main.SCREEN_WIDTH,Main.SCREEN_HEIGHT);
			screenGraphic = screenImage.getGraphics();
			screenDraw(screenGraphic);
			g.drawImage(screenImage,0,0,null);
		}
		
		public void screenDraw(Graphics g)
		{
			g.drawImage(introBackground,0,0,null);
			this.repaint();
			
		}

}
