varying float x;
varying float y;
uniform sampler2D txtr;

void main() {
	float dx = 1.0/320.0;
	float dy = 1.0/240.0;
	
	vec4 col = texture(txtr,vec2(x,y),0.0);
	
	gl_FragColor = col;
	//TOP
	float yTemp = y;
	vec4 colTemp = texture(txtr,vec2(x,yTemp),0.0);
	if (y < 0.5) {
		while (true) {
			if ( !(col.x == colTemp.x && col.y == colTemp.y && col.z == colTemp.z) &&
				 !(colTemp.x == 0 && colTemp.y == 0 && colTemp.y == 0) && 
				 !(colTemp.x == 1 && colTemp.y == 1 && colTemp.y == 1) ) {
				gl_FragColor = vec4(0,0,0,1);
				break;
			} else {
				if ( yTemp < 0.5 ) {
					yTemp = yTemp + dy;
					colTemp = texture(txtr,vec2(x,yTemp),0.0);
				} else {
					break;
				}
			}
		}
	}
}