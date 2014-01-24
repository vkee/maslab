varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	float horizon = 0.52;
	
	// BALL DETECTION RED
	int r = 1;
	int miss = 0;	
	while ( r < 50 ) {
		miss = 0;
		float start = sin(r+x*y)*3.14;
		for ( float a = start; a < start+6.28; a+=0.5 ) {
			vec4 col = texture(txtr,vec2(x+float(r)*cos(a)*dx,y+float(r)*sin(a)*dy),0.0);
			if ( !(col.z==1 && col.y==0 && col.x==0) ) {
				miss++;
			}
		}
		if ( miss>=4 ) {
			break;
		}
		r++;
	}
	if ( miss>=8 && r>=4 && y > 0.5) {
		gl_FragColor = vec4(0,0,r/50.0,1);
	}
		
	// BALL DETECTION GREEN
	r = 1;
	miss = 0;	
	while ( r < 50 ) {
		miss = 0;
		float start = sin(r+x*y)*3.14;
		for ( float a = start; a < start+6.28; a+=0.5 ) {
			vec4 col = texture(txtr,vec2(x+float(r)*cos(a)*dx,y+float(r)*sin(a)*dy),0.0);
			if ( !(col.z==0 && col.y==1 && col.x==0) ) {
				miss++;
			}
		}
		if ( miss>=4 ) {
			break;
		}
		r++;
	}
	if ( miss>=8 && r>=4 && y > 0.5)
		gl_FragColor = vec4(0,r/50.0,0,1);
	
	// WALL DETECTION
	
	float yUpper = y;
	float yLower = y;
	vec4 col = texture(txtr,vec2(x,y),0.0);
	vec4 colUpper = texture(txtr,vec2(x,yUpper),0.0);
	vec4 colLower = texture(txtr,vec2(x,yLower),0.0);
	
	//BLUE
	if ( col.z==0 && col.y==0 && col.x==1 && y<horizon ) {
		while ( colUpper.z==0 && colUpper.y==0 && colUpper.x==1 && yUpper > 0) {
			yUpper = yUpper - dy;
			colUpper = texture(txtr,vec2(x,yUpper),0.0);
		}
		
		while ( colLower.z==0 && colLower.y==0 && colLower.x==1 && yLower < 1) {
			yLower = yLower + dy;
			colLower = texture(txtr,vec2(x,yLower),0.0);
		}
		
		gl_FragColor = vec4((yLower - yUpper),0,0,1);
	}
	
	float yLeft1 = y;
	float yLeft2 = y;
	float yLeft3 = y;
	float xLeft = x;
	vec4 colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
	vec4 colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
	vec4 colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);	
	float yRight1 = y;
	float yRight2 = y;
	float yRight3 = y;
	float xRight = x;
	vec4 colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
	vec4 colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
	vec4 colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
	
	
	//YELLOW
	if ( col.z==1 && col.y==1 && col.x==0 && y<horizon ) {
		while ( colUpper.z==1 && colUpper.y==1 && colUpper.x==0 && yUpper > 0) {
			yUpper = yUpper - dy;
			colUpper = texture(txtr,vec2(x,yUpper),0.0);
		}
		
		while ( colLower.z==1 && colLower.y==1 && colLower.x==0 && yLower < 1) {
			yLower = yLower + dy;
			colLower = texture(txtr,vec2(x,yLower),0.0);
		}
		
		if ( yUpper+yLower==2*y || yUpper+yLower==2*y+dy || yUpper+yLower==2*y-dy) {
			while (true) {
				if ( colLeft2.z==1 && colLeft2.y==1 && colLeft2.x==0 && yLeft2 < 1 && yLeft2 > 0 && xLeft > 0) {
					yLeft1 = yLeft2 + dy;
					yLeft2 = yLeft2;
					yLeft3 = yLeft2 - dy;
					xLeft = xLeft - dx;
					colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
					colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
					colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);
				} else if ( colLeft1.z==1 && colLeft1.y==1 && colLeft1.x==0 && yLeft1 < 1 && yLeft1 > 0 && xLeft > 0) {
					yLeft1 = yLeft1 + dy;
					yLeft2 = yLeft1;
					yLeft3 = yLeft1 - dy;
					xLeft = xLeft - dx;
					colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
					colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
					colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);
				} else if ( colLeft3.z==1 && colLeft3.y==1 && colLeft3.x==0 && yLeft3 < 1 && yLeft3 > 0 && xLeft > 0) {
					yLeft1 = yLeft3 + dy;
					yLeft2 = yLeft3;
					yLeft3 = yLeft3 - dy;
					xLeft = xLeft - dx;
					colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
					colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
					colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);
				} else {
					break;
				}
				
			}
			
			while (true) {
				if ( colRight2.z==1 && colRight2.y==1 && colRight2.x==0 && yRight2 < 1 && yRight2 > 0 && xRight < 1) {
					yRight1 = yRight2 + dy;
					yRight2 = yRight2;
					yRight3 = yRight2 - dy;
					xRight = xRight + dx;
					colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
					colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
					colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
				} else if ( colRight1.z==1 && colRight1.y==1 && colRight1.x==0 && yRight1 < 1 && yRight1 > 0 && xRight < 1) {
					yRight1 = yRight1 + dy;
					yRight2 = yRight1;
					yRight3 = yRight1 - dy;
					xRight = xRight + dx;
					colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
					colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
					colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
				} else if ( colRight3.z==1 && colRight3.y==1 && colRight3.x==0 && yRight3 < 1 && yRight3 > 0 && xRight < 1) {
					yRight1 = yRight3 + dy;
					yRight2 = yRight3;
					yRight3 = yRight3 - dy;
					xRight = xRight + dx;
					colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
					colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
					colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
				} else {
					break;
				}
				
			}
			
			if ( xLeft+xRight <= 2*x+dx && xLeft+xRight >= 2*x-dx ) {
				gl_FragColor = vec4(0,(yLower - yUpper),(yLower - yUpper),1);
			}
		} 
	}
	
	//RED
	if ( col.z==1 && col.y==0 && col.x==0 && y<horizon ) {
		while ( colUpper.z==1 && colUpper.y==0 && colUpper.x==0 && yUpper > 0) {
			yUpper = yUpper - dy;
			colUpper = texture(txtr,vec2(x,yUpper),0.0);
		}
		
		while ( colLower.z==1 && colLower.y==0 && colLower.x==0 && yLower < 1) {
			yLower = yLower + dy;
			colLower = texture(txtr,vec2(x,yLower),0.0);
		}
		
		if ( yUpper+yLower==2*y || yUpper+yLower==2*y+dy || yUpper+yLower==2*y-dy) {
			while (true) {
				if ( colLeft2.z==1 && colLeft2.y==0 && colLeft2.x==0 && yLeft2 < 1 && yLeft2 > 0 && xLeft > 0) {
					yLeft1 = yLeft2 + dy;
					yLeft2 = yLeft2;
					yLeft3 = yLeft2 - dy;
					xLeft = xLeft - dx;
					colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
					colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
					colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);
				} else if ( colLeft1.z==1 && colLeft1.y==0 && colLeft1.x==0 && yLeft1 < 1 && yLeft1 > 0 && xLeft > 0) {
					yLeft1 = yLeft1 + dy;
					yLeft2 = yLeft1;
					yLeft3 = yLeft1 - dy;
					xLeft = xLeft - dx;
					colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
					colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
					colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);
				} else if ( colLeft3.z==1 && colLeft3.y==0 && colLeft3.x==0 && yLeft3 < 1 && yLeft3 > 0 && xLeft > 0) {
					yLeft1 = yLeft3 + dy;
					yLeft2 = yLeft3;
					yLeft3 = yLeft3 - dy;
					xLeft = xLeft - dx;
					colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
					colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
					colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);
				} else {
					break;
				}
				
			}
			
			while (true) {
				if ( colRight2.z==1 && colRight2.y==0 && colRight2.x==0 && yRight2 < 1 && yRight2 > 0 && xRight < 1) {
					yRight1 = yRight2 + dy;
					yRight2 = yRight2;
					yRight3 = yRight2 - dy;
					xRight = xRight + dx;
					colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
					colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
					colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
				} else if ( colRight1.z==1 && colRight1.y==0 && colRight1.x==0 && yRight1 < 1 && yRight1 > 0 && xRight < 1) {
					yRight1 = yRight1 + dy;
					yRight2 = yRight1;
					yRight3 = yRight1 - dy;
					xRight = xRight + dx;
					colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
					colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
					colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
				} else if ( colRight3.z==1 && colRight3.y==0 && colRight3.x==0 && yRight3 < 1 && yRight3 > 0 && xRight < 1) {
					yRight1 = yRight3 + dy;
					yRight2 = yRight3;
					yRight3 = yRight3 - dy;
					xRight = xRight + dx;
					colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
					colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
					colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
				} else {
					break;
				}
				
			}
			
			if ( xLeft+xRight <= 2*x+dx && xLeft+xRight >= 2*x-dx ) {
				gl_FragColor = vec4(0,0,(yLower - yUpper),1);
			} 
		} 
	}
	
	//GREEN
	if ( col.z==0 && col.y==1 && col.x==0 && y<horizon ) {
		while ( colUpper.z==0 && colUpper.y==1 && colUpper.x==0 && yUpper > 0) {
			yUpper = yUpper - dy;
			colUpper = texture(txtr,vec2(x,yUpper),0.0);
		}
		
		while ( colLower.z==0 && colLower.y==1 && colLower.x==0 && yLower < 1) {
			yLower = yLower + dy;
			colLower = texture(txtr,vec2(x,yLower),0.0);
		}
		
		if ( yUpper+yLower==2*y || yUpper+yLower==2*y+dy || yUpper+yLower==2*y-dy) {
			while (true) {
				if ( colLeft2.z==0 && colLeft2.y==1 && colLeft2.x==0 && yLeft2 < 1 && yLeft2 > 0 && xLeft > 0) {
					yLeft1 = yLeft2 + dy;
					yLeft2 = yLeft2;
					yLeft3 = yLeft2 - dy;
					xLeft = xLeft - dx;
					colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
					colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
					colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);
				} else if ( colLeft1.z==0 && colLeft1.y==1 && colLeft1.x==0 && yLeft1 < 1 && yLeft1 > 0 && xLeft > 0) {
					yLeft1 = yLeft1 + dy;
					yLeft2 = yLeft1;
					yLeft3 = yLeft1 - dy;
					xLeft = xLeft - dx;
					colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
					colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
					colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);
				} else if ( colLeft3.z==0 && colLeft3.y==1 && colLeft3.x==0 && yLeft3 < 1 && yLeft3 > 0 && xLeft > 0) {
					yLeft1 = yLeft3 + dy;
					yLeft2 = yLeft3;
					yLeft3 = yLeft3 - dy;
					xLeft = xLeft - dx;
					colLeft1 = texture(txtr,vec2(xLeft,yLeft1),0.0);
					colLeft2 = texture(txtr,vec2(xLeft,yLeft2),0.0);
					colLeft3 = texture(txtr,vec2(xLeft,yLeft3),0.0);
				} else {
					break;
				}
				
			}
			
			while (true) {
				if ( colRight2.z==0 && colRight2.y==1 && colRight2.x==0 && yRight2 < 1 && yRight2 > 0 && xRight < 1) {
					yRight1 = yRight2 + dy;
					yRight2 = yRight2;
					yRight3 = yRight2 - dy;
					xRight = xRight + dx;
					colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
					colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
					colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
				} else if ( colRight1.z==0 && colRight1.y==1 && colRight1.x==0 && yRight1 < 1 && yRight1 > 0 && xRight < 1) {
					yRight1 = yRight1 + dy;
					yRight2 = yRight1;
					yRight3 = yRight1 - dy;
					xRight = xRight + dx;
					colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
					colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
					colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
				} else if ( colRight3.z==0 && colRight3.y==1 && colRight3.x==0 && yRight3 < 1 && yRight3 > 0 && xRight < 1) {
					yRight1 = yRight3 + dy;
					yRight2 = yRight3;
					yRight3 = yRight3 - dy;
					xRight = xRight + dx;
					colRight1 = texture(txtr,vec2(xRight,yRight1),0.0);
					colRight2 = texture(txtr,vec2(xRight,yRight2),0.0);
					colRight3 = texture(txtr,vec2(xRight,yRight3),0.0);
				} else {
					break;
				}
				
			}
			
			if ( xLeft+xRight <= 2*x+dx && xLeft+xRight >= 2*x-dx ) {
				gl_FragColor = vec4(0,(yLower - yUpper),0,1);
			}
		} 
	}
}