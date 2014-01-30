package BotClient;

public class GameStartTest {
	public static void main( String[] args ) {
		
		BotClient botclient = new BotClient("18.150.7.174:6667","b3MpHHs4J1",false);
		
		while( !botclient.gameStarted() ) {
		}
		System.out.println("***GAME STARTED***");
		System.out.println("MAP --> " + botclient.getMap());
		
		botclient.close();
	}
}
