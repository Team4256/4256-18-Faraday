package org.usfirst.frc.team4256.robot.Autonomous;

public class V_Instructions {
	public static final double leftStartX = -112.33, centerStartX = 7.19, rightStartX = 115.0;
	public static final double switchX = 95.43, cubeX = 72.64, scaleX = 83.11;
	public static final double startY = 13.32, switchY = 166.6, cubeY = 214.35, scaleY = 296.13;
	
	public final double initOdometerPosX;
	
	public enum StartingPosition {LEFT, CENTER, RIGHT};
	public enum FieldPieceConfig {LEFT, RIGHT};
	
	public final FieldPieceConfig switchTarget;
	public final FieldPieceConfig scaleTarget;
	public final StartingPosition startingPosition;
	
	private V_Leash leash;
	
	public V_Instructions(final int startingPosition) {
		switchTarget = null;
		scaleTarget = null;
		switch (startingPosition) {//ROBOT
		case(0):this.startingPosition = StartingPosition.LEFT;break;
		case(1):this.startingPosition = StartingPosition.CENTER;break;
		case(2):this.startingPosition = StartingPosition.RIGHT;break;
		default: this.startingPosition = StartingPosition.CENTER;break;
		}
		
		switch (this.startingPosition) {
		case LEFT:initOdometerPosX = leftStartX;
			final P_Bezier l = new P_Bezier(leftStartX, startY, -110, 93, -93, 93, -switchX, switchY, 0.0);//get to nearest switch
			P_Bezier[] pl = new P_Bezier[] {l};
			leash = new V_Leash(pl, /*leash length*/3.0, /*growth rate*/0.1);
			break;
		case CENTER:initOdometerPosX = centerStartX;
			leash = bezier_center2switchLeft();//get to left switch, might as well be random side though
			break;
		case RIGHT:initOdometerPosX = rightStartX;
			final P_Bezier r = new P_Bezier(rightStartX, startY, 116, 89, 99, 89, switchX, switchY, 0.0);//get to nearest switch
			P_Bezier[] pr = new P_Bezier[] {r};
			leash = new V_Leash(pr, /*leash length*/3.0, /*growth rate*/0.1);
			break;
		default:initOdometerPosX = centerStartX;break;
		}
		
	}
	
	public V_Instructions(final String gameData, final int startingPosition) {
		//{organize initialization data}
		switchTarget = gameData.charAt(0) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SWITCH
		scaleTarget = gameData.charAt(1) == 'L' ? FieldPieceConfig.LEFT : FieldPieceConfig.RIGHT;//SCALE
		switch (startingPosition) {//ROBOT
		case(0):this.startingPosition = StartingPosition.LEFT;break;
		case(1):this.startingPosition = StartingPosition.CENTER;break;
		case(2):this.startingPosition = StartingPosition.RIGHT;break;
		default: this.startingPosition = StartingPosition.CENTER;break;
		}
		
		switch (this.startingPosition) {//TODO for each starting pos, have one set of events for paths of 3 and one for paths of 2
		case LEFT:initOdometerPosX = leftStartX;
		
			if (switchTarget.equals(FieldPieceConfig.LEFT) && scaleTarget.equals(FieldPieceConfig.LEFT)) {
				//{easy switch then easy scale}//-------------------------------------------------------------------------------------DONE
				final P_Bezier a = new P_Bezier(leftStartX, startY, -110, 93, -93, 93, -switchX, switchY, 0.0);//get to easy switch
				final P_Bezier b = new P_Bezier(-switchX, switchY, -115, 202, -87, 222, -cubeX, cubeY, 1.0);//get to new cube
				final P_Bezier c = new P_Bezier(-cubeX, cubeY, -72.64, 224, -71.11, 286, -scaleX, scaleY, 2.0);//get to easy scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b, c};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else if (switchTarget.equals(FieldPieceConfig.LEFT)) {
				//{easy switch then hard scale}
				final P_Bezier a = new P_Bezier(leftStartX, startY, -110, 93, -93, 93, -switchX, switchY, 0.0);//get to easy switch
				final P_Bezier b = new P_Bezier(-switchX, switchY, -115, 202, -87, 222, -cubeX, cubeY, 1.0);//get to new cube
				final P_Bezier c = new P_Bezier(-cubeX, cubeY, -40, 278, 63, 228, scaleX, scaleY, 2.0);//get to hard scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b, c};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else if (scaleTarget.equals(FieldPieceConfig.LEFT)) {
				//{easy scale then hard switch}//-------------------------------------------------------------------------------------DONE
				final P_Bezier a = new P_Bezier(leftStartX, startY, -119, 206, -92, 238, -scaleX, scaleY, 0.0);//get to easy scale
				final P_Bezier b = new P_Bezier(-scaleX, scaleY, -52, 193, 42, 277, /*cubeX*/64.0, cubeY, 1.0);//get to new cube/hard switch
				
				final P_Bezier[] path = new P_Bezier[] {a, b};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else {
				//{hard switch then hard scale}
				final P_Bezier a = new P_Bezier(leftStartX, startY, -112, 92, -122, 96, -119, 166, 0.0);//get to random place
				final P_Bezier b = new P_Bezier(-119, 166, -117, 297, 141, 126, scaleX, scaleY, 1.0);//pass by hard switch, end at hard scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
			}
			
			break;
			
		case CENTER:initOdometerPosX = centerStartX;
			//{switch then scale}
			//TODO this should be a 2 cube auto
			if (switchTarget.equals(FieldPieceConfig.RIGHT)) leash = bezier_center2switchRight();
			else leash = bezier_center2switchLeft();
			
			break;
		
		case RIGHT:initOdometerPosX = rightStartX;
		
			if (switchTarget.equals(FieldPieceConfig.RIGHT) && scaleTarget.equals(FieldPieceConfig.RIGHT)) {
				//{easy switch then easy scale}//-----------------------------------------------------------------------------------DONE
				final P_Bezier a = new P_Bezier(rightStartX, startY, 116, 89, 99, 89, switchX, switchY, 0.0);//get to easy switch
				final P_Bezier b = new P_Bezier(switchX, switchY, 111, 195, 101, 220, cubeX, cubeY, 1.0);//get to new cube
				final P_Bezier c = new P_Bezier(cubeX, cubeY, 72.64, 224, 71.11, 286, scaleX, scaleY, 2.0);//get to easy scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b, c};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else if (switchTarget.equals(FieldPieceConfig.RIGHT)) {
				//{easy switch then hard scale}
				final P_Bezier a = new P_Bezier(rightStartX, startY, 116, 89, 99, 89, switchX, switchY, 0.0);//get to easy switch
				final P_Bezier b = new P_Bezier(switchX, switchY, 111, 195, 101, 220, cubeX, cubeY, 1.0);//get to new cube
				final P_Bezier c = new P_Bezier(cubeX, cubeY, 40, 278, -63, 228, -scaleX, scaleY, 2.0);//get to hard scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b, c};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
				
			}else if (scaleTarget.equals(FieldPieceConfig.RIGHT)) {
				//{easy scale then hard switch}//-----------------------------------------------------------------------------------DONE
				final P_Bezier a = new P_Bezier(rightStartX, startY, 124, 222, 77, 248, scaleX, scaleY, 0.0);//get to easy scale
				final P_Bezier b = new P_Bezier(scaleX, scaleY, 52, 193, -42, 277, /*-cubeX*/-64.0, cubeY, 1.0);//get to new cube/hard switch
				
				final P_Bezier[] path = new P_Bezier[] {a, b};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
			}else {
				//{hard switch then hard scale}
				final P_Bezier a = new P_Bezier(rightStartX, startY, 116, 76, 113, 90, 119, 166, 0.0);//get to random place
				final P_Bezier b = new P_Bezier(119, 166, 117, 297, -124, 126, -scaleX, scaleY, 1.0);//pass by hard switch, end at hard scale
				
				final P_Bezier[] path = new P_Bezier[] {a, b};
				leash = new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
			}
			
			break;
			
		default:initOdometerPosX = centerStartX;break;
		}
	}
	
	public V_Leash getLeash() {return leash;}
	
	
	

	private static V_Leash bezier_sideRight2scaleRight() {
//		                           p0x     p0y   p1x   p1y  p2x  p2y    p3x     p3y   start
		P_Bezier a = new P_Bezier(67.188, 13.32, 91.8, 126, 108, 252, 129.336, 327.78, 0.0);
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, 3.0, 0.1);
	}
	
	private static V_Leash bezier_sideRight2scaleLeft() {
//                                 p0x     p0y   p1x  p1y  p2x  p2y    p3x    p3y    start
		P_Bezier a = new P_Bezier(67.188, 13.32, 84, 93.6, 99, 187.2, 67.188, 228.36, 0.0);
		P_Bezier b = new P_Bezier(67.188, 228.36, -75.36, 228.36, -121.8, 272.88, -129.336, 327.78, 1.0);
		P_Bezier[] path = new P_Bezier[] {a, b};
		return new V_Leash(path, 3.0, 0.1);
	}
	
	private static V_Leash bezier_sideLeft2scaleRight() {
//                                  p0x     p0y     p1x   p1y    p2x   p2y     p3x      p3y  start
		P_Bezier a = new P_Bezier(-91.812, 13.32, -117.6, 120, -124.8, 252, -129.336, 327.78, 0.0);
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, 3.0, 0.1);
	}
	
	private static V_Leash bezier_sideLeft2scaleLeft() {
//                                  p0x     p0y    p1x   p1y   p2x   p2y     p3x     p3y   start
		P_Bezier a = new P_Bezier(-91.812, 13.32, -108, 82.8, -120, 164.4, -91.812, 228.36, 0.0);
		P_Bezier b = new P_Bezier(-91.812, 228.36, 72, 228.36, 117.6, 265.2, 129.336, 327.78, 1.0);
		P_Bezier[] path = new P_Bezier[] {a, b};
		return new V_Leash(path, 3.0, 0.1);
	}
	
	private static V_Leash bezier_sides2switchNearest() {
		//							p0x   p0y    p1x   p1y   p2x  p2y   p3x   p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 7.188, 40, 7.188, 90, 7.188, 138, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, 3.0, 0.05);
	}
	
	private static V_Leash bezier_center2switchLeft() {
		//							p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 6, 120, -132, 44, -108, 153, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
	
	private static V_Leash bezier_center2switchRight() {
		//							p0x  p0y  p1x  p1y  p2x  p2y  p3x  p3y  start
		P_Bezier a = new P_Bezier(7.188, 13.32, 8, 120, 132, 44, 108, 153, 0.0);//inches
		//Create an array of CubicBeziers; represents a full path.
		P_Bezier[] path = new P_Bezier[] {a};
		return new V_Leash(path, /*leash length*/3.0, /*growth rate*/0.1);
	}
}
