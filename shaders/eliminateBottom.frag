varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	
	vec4 col = texture(txtr,vec2(x,y),0.0);
	float horizon = 0.45;
	gl_FragColor = col;
	
	//STUFF BELOW TEAL
	float yTemp = y;
	vec4 colTemp = texture(txtr,vec2(x,yTemp),0.0);
	float yTempTeal = y;
	int teal = 0;
	if ( y > horizon && 
		((col.x == 0 && col.y == 1 && col.z == 0) || (col.x == 0 && col.y == 0 && col.z == 1)) ) {
		while (true) {
			//check for height of teal strip
			if ( teal == 1 ) {
				if ( !(colTemp.x == 1 && colTemp.y == 1 && colTemp.z == 0) ) {
					break;
				} else {
					if ( yTempTeal > 0 ) {
						yTempTeal = yTempTeal - dy;
						colTemp = texture(txtr,vec2(x,yTempTeal),0.0);
						continue;
					} else {
						break;
					}
				}
			}	
			
			//check for distance between teal
			if ( teal == 0 && colTemp.x == 1 && colTemp.y == 1 && colTemp.z == 0 ) {
				teal = 1;
				yTempTeal = yTemp;
			} else {
				if ( yTemp > 0 ) {
					yTemp = yTemp - dy;
					colTemp = texture(txtr,vec2(x,yTemp),0.0);
				} else {
					break;
				}
			}
		}
		
		if ( teal == 1 && 2.5*(yTemp-yTempTeal)>y-yTemp ) {
			gl_FragColor = vec4(1,1,1,1);
		}
	}
}